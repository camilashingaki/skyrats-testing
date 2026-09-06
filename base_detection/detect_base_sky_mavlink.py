#!/usr/bin/env python3
"""
Base identification + precision landing mission using SkyRats/sky_mavlink
(SkyMAVLink).

The drone arms, takes off, flies forward at a constant body-frame velocity,
and continuously runs a YOLO model (ultralytics, loaded from a `best.pt`
checkpoint) on the live camera feed. As soon as the base is confirmed for a
few consecutive frames, it switches to a centering phase: SkyMAVLink ships no
PID utility, so a small PID is rolled here and wired to the exact continuous-
correction primitive its README recommends -- `set_body_velocity()` re-sent
via `sleep()` -- driving the base's pixel offset from the image center to
zero, descending once well-centered, until it is low enough to hand off to
`drone.land()` for the final touchdown.

SkyMAVLink has no background thread: its message loop only advances inside
blocking calls (`sleep()`, `takeoff()`, `arm()`, ...), so both the search and
centering loops below call `drone.sleep(step)` every iteration -- this both
services the MAVLink link and keeps re-sending the active `set_body_velocity`
setpoint. Camera capture + YOLO inference run on a separate Python thread,
independent of the MAVLink link.

Camera mounting assumption for the centering phase: a forward/nadir camera
where image columns map to the drone's right and image rows map to the
drone's forward (FRD body frame). Flip `--lateral-sign`/`--forward-sign` if
your mount disagrees.

Default connection assumes the flight controller is wired to the Raspberry
Pi's UART pins (`/dev/serial0`, 921600 baud) rather than USB or SITL --
override `--connection` for a different setup. On the Pi, `raspi-config`
must have the serial port hardware enabled and its login shell disabled; on
the FC, the corresponding `SERIALx_PROTOCOL` must be 2 (MAVLink2) and
`SERIALx_BAUD` must match the baud in `--connection`.

Requirements:
    pip install -r requirements.txt
    pip install -e /path/to/sky_mavlink   # SkyMAVLink itself (not on PyPI)

Usage:
    python detect_base_sky_mavlink.py --model models/best.pt
    python detect_base_sky_mavlink.py --connection serial:/dev/ttyAMA0:57600 --model models/best.pt
    python detect_base_sky_mavlink.py --connection tcp:127.0.0.1:5760 --model models/best.pt   # SITL
"""

import argparse
import logging
import threading
import time
from typing import Optional, Set

import cv2
from skymavlink import SkyMAVLink
from ultralytics import YOLO

log = logging.getLogger("base_detection_sky_mavlink")


class Detector:
    """Minimal ultralytics YOLO wrapper mirroring the fields used below."""

    def __init__(self, model_path: str) -> None:
        self.model = YOLO(model_path)

    @property
    def class_names(self):
        return self.model.names

    def detect(self, frame, conf: float):
        results = self.model.predict(frame, conf=conf, verbose=False)
        return results[0]

    def draw(self, result):
        return result.plot()


class DetectionState:
    """Shared, lock-protected view of the latest detector output.

    Written from the camera thread; read from the mission thread.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame = None
        self.result = None
        self.consecutive_hits = 0


class SimplePID:
    """Minimal PID controller.

    SkyMAVLink ships no PID utility of its own, so this mirrors the same
    discrete-time PID math (proportional + clamped integral + derivative,
    output clamped) that nectar.control.pid.PIDController documents, wired
    here to SkyMAVLink's own recommended continuous-correction primitive:
    `set_body_velocity()` re-sent via `sleep()`.
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


def best_candidate(result, class_names, target_classes: Optional[Set[str]]):
    """Highest-confidence detection matching `target_classes` (or None)."""
    if result is None or result.boxes is None or len(result.boxes) == 0:
        return None

    best_idx, best_conf = None, -1.0
    for i, (cls_id, conf) in enumerate(zip(result.boxes.cls.tolist(), result.boxes.conf.tolist())):
        name = class_names[int(cls_id)]
        if target_classes is not None and name not in target_classes:
            continue
        if conf > best_conf:
            best_idx, best_conf = i, conf

    if best_idx is None:
        return None

    xyxy = result.boxes.xyxy[best_idx].tolist()
    cls_id = int(result.boxes.cls[best_idx].item())
    return {
        "class_name": class_names[cls_id],
        "confidence": best_conf,
        "xyxy": xyxy,
        "center": ((xyxy[0] + xyxy[2]) / 2.0, (xyxy[1] + xyxy[3]) / 2.0),
    }


def matches_target(result, class_names, target_classes: Optional[Set[str]]) -> bool:
    return best_candidate(result, class_names, target_classes) is not None


def report_detection(detector: Detector, state: DetectionState, target_classes, snapshot_path: str) -> None:
    with state.lock:
        result = state.result
        frame = state.frame

    best = best_candidate(result, detector.class_names, target_classes)
    if best is None:
        return
    log.info("BASE FOUND -> class=%s conf=%.2f bbox=%s", best["class_name"], best["confidence"], best["xyxy"])

    if frame is not None:
        annotated = detector.draw(result)
        cv2.imwrite(snapshot_path, annotated)
        log.info("Saved detection snapshot to %s", snapshot_path)


def center_and_land(drone, detector: Detector, state: DetectionState, target_classes, args: argparse.Namespace) -> None:
    """Visually servo onto the base, descending once centered, then land.

    Drives the base's normalized pixel offset from the image center to zero
    via SkyMAVLink's `set_body_velocity(fwd, right, down)`, re-sent each loop
    iteration through `sleep()` as its README's "ending a velocity leg" /
    continuous-correction pattern recommends. Descends only while centered;
    hands off to `land()` once low enough or if the target is lost too long.
    """
    class_names = detector.class_names
    pid_lateral = SimplePID(args.kp, args.ki, args.kd, args.max_correction)
    pid_forward = SimplePID(args.kp, args.ki, args.kd, args.max_correction)

    lost_frames = 0
    start = time.time()
    while time.time() - start < args.landing_timeout:
        with state.lock:
            result = state.result
            frame = state.frame

        best = best_candidate(result, class_names, target_classes)
        if best is None or frame is None:
            lost_frames += 1
            if lost_frames > args.max_lost_frames:
                log.warning("Target lost during final approach; landing at current position.")
                return
            drone.set_body_velocity(0.0, 0.0, 0.0)
            drone.sleep(args.step)
            continue
        lost_frames = 0

        h, w = frame.shape[:2]
        cx, cy = best["center"]
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


def camera_loop(cap, detector: Detector, state: DetectionState, conf: float, target_classes, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            continue
        result = detector.detect(frame, conf=conf)
        with state.lock:
            state.frame = frame
            state.result = result
            if matches_target(result, detector.class_names, target_classes):
                state.consecutive_hits += 1
            else:
                state.consecutive_hits = 0


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Fly forward and identify the base (SkyMAVLink)")
    parser.add_argument(
        "--connection",
        default="serial:/dev/serial0:921600",
        help="SkyMAVLink endpoint. Default assumes the FC is on the Pi's UART pins. "
        "Also accepts tcp:127.0.0.1:5760 (SITL) or udpout:HOST:PORT (behind a router).",
    )
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera device index.")
    parser.add_argument("--model", default="models/best.pt", help="Path to the YOLO best.pt weights.")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum detection confidence.")
    parser.add_argument(
        "--classes",
        nargs="*",
        default=None,
        help="Class names that count as 'base'. Default: accept any detected class.",
    )
    parser.add_argument("--confirm-frames", type=int, default=3, help="Consecutive positive frames required to confirm.")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude (m).")
    parser.add_argument("--speed", type=float, default=0.3, help="Forward speed (m/s).")
    parser.add_argument("--step", type=float, default=0.1, help="Seconds serviced per search-loop iteration.")
    parser.add_argument("--search-timeout", type=float, default=60.0, help="Max seconds spent searching before giving up.")
    parser.add_argument("--snapshot", default="base_detected.jpg", help="Where to save the annotated detection frame.")

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
    args = parser.parse_args()

    target_classes = set(args.classes) if args.classes else None

    detector = Detector(args.model)
    log.info("Loaded model %s, classes=%s", args.model, detector.class_names)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        log.error("Failed to open camera index %d", args.camera_index)
        return

    state = DetectionState()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=camera_loop,
        args=(cap, detector, state, args.conf, target_classes, stop_event),
        daemon=True,
    )
    cam_thread.start()

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    found = False
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm()
        drone.takeoff(args.height)

        north, east, down = drone.wait_for_position()
        log.info("Airborne at N=%.2f E=%.2f D=%.2f", north, east, down)

        log.info("Flying forward at %.2f m/s, searching for the base...", args.speed)
        drone.set_body_velocity(args.speed, 0.0, 0.0)

        start = time.time()
        while time.time() - start < args.search_timeout:
            with state.lock:
                hits = state.consecutive_hits
            if hits >= args.confirm_frames:
                found = True
                break
            drone.sleep(args.step)  # services the link and re-sends the velocity setpoint

        drone.set_body_velocity(0.0, 0.0, 0.0)  # brake
        drone.sleep(2.0)

        if found:
            report_detection(detector, state, target_classes, args.snapshot)
            log.info("Centering on the base and descending to land on top of it...")
            center_and_land(drone, detector, state, target_classes, args)
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


if __name__ == "__main__":
    main()
