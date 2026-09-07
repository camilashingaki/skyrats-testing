"""
Shared search-square + PID-centering + landing logic for the precision
landing missions (precision_landing_ncnn.py / precision_landing_tflite.py).
The only difference between those two entry points is which model format
loads by default; RawDetector picks the right backend from the path either
way, so both call run() with a different default --model.

Builds on:
  - base_detection/raw_detector.py -- torch-free NCNN/TFLite inference,
    already returns a single best-match centroid (no bbox needed here).
  - base_detection/detect_base_sky_mavlink.py -- the PID-centering /
    landing phase is reused close to unchanged from there (same PID,
    same set_body_velocity-driven visual servo, same land-altitude
    handoff); what's new here is a locked-heading search **square**
    instead of a single straight forward leg, per skymavlink/CLAUDE.md's
    "yaw fixed" rule for body-frame patterns.
  - skymavlink -- flight control (arm/takeoff/set_body_velocity/land).
"""

import argparse
import logging
import math
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Set

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
BASE_DETECTION_DIR = REPO_ROOT / "base_detection"

# base_detection/ has no __init__.py -- it's a folder of standalone scripts,
# not a package -- so raw_detector is imported by adding its directory to
# sys.path rather than a package-relative import.
if str(BASE_DETECTION_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DETECTION_DIR))

from raw_detector import RawDetector  # noqa: E402

from skymavlink import SkyMAVLink

log = logging.getLogger("precision_landing")

ENDPOINT_SITL = "tcp:127.0.0.1:5760"
DEFAULT_NCNN_MODEL = BASE_DETECTION_DIR / "models" / "best_ncnn_model"
DEFAULT_TFLITE_MODEL = BASE_DETECTION_DIR / "models" / "best_w8a32.tflite"


class DetectionState:
    """Shared, lock-protected view of the latest detector output.

    Written from the camera thread; read from the mission thread. SkyMAVLink
    runs no background thread of its own -- its message loop only advances
    inside blocking calls -- so camera capture + inference live on a
    separate Python thread and hand results across this lock.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame = None
        self.result = None
        self.consecutive_hits = 0


class SimplePID:
    """Minimal PID controller -- SkyMAVLink ships no PID utility of its own.

    Mirrors nectar.control.pid.PIDController's documented "usable standalone
    for any control loop" primitive, wired to SkyMAVLink's own recommended
    continuous-correction primitive: set_body_velocity() re-sent via sleep().
    """

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time: Optional[float] = None

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self._last_time if self._last_time is not None else 0.0
        self._last_time = now

        self._integral = max(-self.output_limit, min(self.output_limit, self._integral + error * dt))
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        self._last_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


def _matches(result, target_classes: Optional[Set[str]]) -> bool:
    return result is not None and (target_classes is None or result["class_name"] in target_classes)


def camera_loop(cap, detector: RawDetector, state: DetectionState, target_classes, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            continue
        result = detector.detect(frame)
        with state.lock:
            state.frame = frame
            state.result = result
            state.consecutive_hits = state.consecutive_hits + 1 if _matches(result, target_classes) else 0


def report_detection(state: DetectionState, target_classes, snapshot_path: str, detector: RawDetector) -> None:
    with state.lock:
        result, frame = state.result, state.frame
    if not _matches(result, target_classes) or frame is None:
        return
    log.info("BASE FOUND -> class=%s conf=%.2f center=%s", result["class_name"], result["confidence"], result["center"])
    cv2.imwrite(snapshot_path, detector.draw(frame, result))
    log.info("Saved detection snapshot to %s", snapshot_path)


def center_and_land(drone: SkyMAVLink, state: DetectionState, target_classes, args: argparse.Namespace) -> None:
    """Visually servo onto the base, descending once centered, then land.

    Drives the base's normalized pixel offset from the image center to zero
    via set_body_velocity(fwd, right, down), re-sent every loop iteration
    through sleep(). Descends only while centered; hands off to land() once
    low enough or if the target is lost too long.
    """
    pid_lateral = SimplePID(args.kp, args.ki, args.kd, args.max_correction)
    pid_forward = SimplePID(args.kp, args.ki, args.kd, args.max_correction)

    lost_frames = 0
    start = time.time()
    while time.time() - start < args.landing_timeout:
        with state.lock:
            result, frame = state.result, state.frame

        if not _matches(result, target_classes) or frame is None:
            lost_frames += 1
            if lost_frames > args.max_lost_frames:
                log.warning("Target lost during final approach; landing at current position.")
                return
            drone.set_body_velocity(0.0, 0.0, 0.0)
            drone.sleep(args.step)
            continue
        lost_frames = 0

        h, w = frame.shape[:2]
        cx, cy = result["center"]
        err_x = (cx - w / 2.0) / (w / 2.0)  # [-1, 1], + = target to the right
        err_y = (cy - h / 2.0) / (h / 2.0)  # [-1, 1], + = target below center

        right = args.lateral_sign * pid_lateral.update(err_x)
        fwd = args.forward_sign * pid_forward.update(err_y)

        centered = abs(err_x) < args.align_tolerance and abs(err_y) < args.align_tolerance
        down = args.descent_speed if centered else 0.0  # FRD: down positive = descend

        pose = drone.get_pose_local()
        altitude = -pose[2] if pose is not None else None
        if altitude is not None and altitude <= args.land_altitude:
            log.info("Centered and low (%.2fm); handing off to land().", altitude)
            return

        drone.set_body_velocity(fwd, right, down)
        drone.sleep(args.step)

    log.warning("Landing phase timed out after %.0fs; landing at current position.", args.landing_timeout)


def search_square(drone: SkyMAVLink, state: DetectionState, args: argparse.Namespace) -> bool:
    """Fly a locked-heading square, scanning for the base on every leg.

    Uses set_body_velocity() rather than set_body_pose(): a position command
    would have to be re-issued to react once a detection lands mid-leg, a
    velocity setpoint just gets overwritten with zero. Heading stays locked
    the whole time because set_body_velocity() always sends an explicit
    yaw_rate (0 here) -- see skymavlink/CLAUDE.md, "Never flag yaw_rate
    ignore on a velocity setpoint" -- so the search never turns to face a
    corner the way ArduPilot's default WP_YAW_BEHAVIOR otherwise would.

    Distance per leg is measured from get_pose_local(), never from
    speed * time, so wind/battery sag can't silently shrink or grow the
    square. Returns True the instant --confirm-frames consecutive hits land.
    """
    side, speed = args.side, args.speed
    legs = [(speed, 0.0), (0.0, speed), (-speed, 0.0), (0.0, -speed)]
    leg_time_budget = side / speed + 5.0  # fail-safe only; arrival is distance-gated

    start = time.time()
    for i, (vx, vy) in enumerate(legs, start=1):
        pose = drone.get_pose_local()
        leg_start_n, leg_start_e = pose[0], pose[1]
        log.info("Search leg %d/4: vx=%.2f vy=%.2f", i, vx, vy)
        drone.set_body_velocity(vx, vy, 0.0)

        leg_elapsed = 0.0
        while leg_elapsed < leg_time_budget and time.time() - start < args.search_timeout:
            with state.lock:
                hits = state.consecutive_hits
            if hits >= args.confirm_frames:
                drone.set_body_velocity(0.0, 0.0, 0.0)
                drone.sleep(2.0)
                return True

            drone.sleep(args.step)
            leg_elapsed += args.step

            pose = drone.get_pose_local()
            if pose is not None and math.hypot(pose[0] - leg_start_n, pose[1] - leg_start_e) >= side:
                break

        if time.time() - start >= args.search_timeout:
            break

    drone.set_body_velocity(0.0, 0.0, 0.0)
    drone.sleep(2.0)
    return False


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera device index.")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum detection confidence.")
    parser.add_argument(
        "--classes", nargs="*", default=None,
        help="Class names counted as 'base' (e.g. shape_hexagon). Default: any shape class RawDetector matches.",
    )
    parser.add_argument("--confirm-frames", type=int, default=3, help="Consecutive positive frames required before confirming.")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude (m).")
    parser.add_argument("--side", type=float, default=3.0, help="Search-square side length (m).")
    parser.add_argument("--speed", type=float, default=0.3, help="Search speed (m/s) on each leg.")
    parser.add_argument("--step", type=float, default=0.1, help="Seconds serviced per search-loop iteration.")
    parser.add_argument("--search-timeout", type=float, default=90.0, help="Max seconds spent searching before giving up.")
    parser.add_argument("--snapshot", default="base_detected.jpg", help="Where to save the annotated detection frame.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")

    # Precision-landing / centering phase.
    parser.add_argument("--align-tolerance", type=float, default=0.15, help="Normalized pixel error (0-1) below which the base counts as centered.")
    parser.add_argument("--land-altitude", type=float, default=0.4, help="Altitude (m) at which centering hands off to land().")
    parser.add_argument("--descent-speed", type=float, default=0.15, help="Descent speed (m/s) while centered.")
    parser.add_argument("--landing-timeout", type=float, default=45.0, help="Max seconds spent centering/descending before landing anyway.")
    parser.add_argument("--max-lost-frames", type=int, default=15, help="Frames without a matching detection tolerated during descent.")
    parser.add_argument("--kp", type=float, default=0.6, help="PID proportional gain for both centering axes.")
    parser.add_argument("--ki", type=float, default=0.0, help="PID integral gain for both centering axes.")
    parser.add_argument("--kd", type=float, default=0.05, help="PID derivative gain for both centering axes.")
    parser.add_argument("--max-correction", type=float, default=0.4, help="Max horizontal correction speed (m/s) during centering.")
    parser.add_argument("--lateral-sign", type=int, choices=[1, -1], default=1, help="Flip if the drone drifts away from the base sideways.")
    parser.add_argument("--forward-sign", type=int, choices=[1, -1], default=1, help="Flip if the drone drifts away from the base forward/back.")


def run(default_model: str, backend_label: str) -> None:
    """Entry point shared by precision_landing_ncnn.py and _tflite.py."""
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description=f"Fly a search square and precision-land on the base ({backend_label}).")
    parser.add_argument("--model", default=default_model, help=f"RawDetector model path (default: {backend_label}).")
    add_common_args(parser)
    args = parser.parse_args()

    target_classes = set(args.classes) if args.classes else None

    detector = RawDetector(args.model, conf=args.conf)
    log.info("Loaded %s detector %s, classes=%s", backend_label, args.model, detector.class_names)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        log.error("Failed to open camera index %d", args.camera_index)
        return

    state = DetectionState()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=camera_loop, args=(cap, detector, state, target_classes, stop_event), daemon=True)
    cam_thread.start()

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    found = False
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.height)

        north, east, down = drone.wait_for_position()
        log.info("Airborne at N=%.2f E=%.2f D=%.2f", north, east, down)

        log.info("Searching for the base on a %.1fm square at %.2f m/s...", args.side, args.speed)
        found = search_square(drone, state, args)

        if found:
            report_detection(state, target_classes, args.snapshot, detector)
            log.info("Centering on the base and descending to land on top of it...")
            center_and_land(drone, state, target_classes, args)
        else:
            log.warning("Base not identified within %.0fs; landing at current position.", args.search_timeout)
    except TimeoutError as e:
        log.error("Timeout during mission: %s", e)
        drone.rtl()
    except KeyboardInterrupt:
        log.info("Interrupted -- returning to launch")
        drone.rtl()
    finally:
        if drone.is_armed():
            drone.land()
        stop_event.set()
        cam_thread.join(timeout=2.0)
        cap.release()
        drone.shutdown()
