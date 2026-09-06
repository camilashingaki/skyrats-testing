#!/usr/bin/env python3
"""
Base identification + precision landing mission using the Nectar SDK
(Black-Bee-Drones/nectar-sdk).

The drone takes off, flies forward at a constant velocity, and continuously
runs a YOLO model (nectar.ai.detection.Detector, loaded from a `best.pt`
checkpoint) on the live camera feed. As soon as the base is confirmed for a
few consecutive frames, it switches to a centering phase: two
`nectar.control.pid.PIDController` instances (Nectar's own generic control
loop primitive) drive the base's pixel offset from the image center to zero
via `move_velocity(reference=BODY)`, descending once well-centered, until it
is low enough to hand off to `drone.land()` for the final touchdown.

Camera mounting assumption for the centering phase: a forward/nadir camera
where image columns map to the drone's right (body Y) and image rows map to
the drone's forward (body X). Flip `--lateral-sign`/`--forward-sign` if your
mount disagrees.

Requirements:
    pip install nectar-sdk opencv-python ultralytics

Usage:
    python detect_base_nectar.py --model models/best.pt --drone mavlink
    python detect_base_nectar.py --model models/best.pt --drone mavros --env indoor
    python detect_base_nectar.py --model models/best.pt --camera-type ros --classes base
"""

import argparse
import logging
import threading
import time
from typing import Optional, Set

import cv2

import nectar
from nectar.ai.detection import Detector
from nectar.control import (
    DroneFactory,
    MavlinkConfig,
    MavrosConfig,
    MoveReference,
    PoseSource,
    Px4DdsConfig,
    Px4MavlinkConfig,
    Px4MavrosConfig,
)
from nectar.control.pid import PIDController
from nectar.vision.camera import ImageHandler
from nectar.vision.camera.config_builder import ConfigBuilder

log = logging.getLogger("base_detection_nectar")

_CAMERA_PARAMS = {
    "webcam": {"device_index": 0, "width": 1280, "height": 720, "fps": 30},
    "imx219": {"sensor_id": 0, "width": 1280, "height": 720, "flip": 2},
    "ros": {"topic": "/camera/color/image_raw/compressed", "compressed": True},
}


class DetectionState:
    """Shared, lock-protected view of the latest detector output.

    Written from the camera's background executor thread (ImageHandler's
    per-frame callback); read from the main mission thread.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame = None
        self.result = None
        self.consecutive_hits = 0


def build_drone_config(args: argparse.Namespace):
    pose_source = PoseSource.VISION if args.env == "indoor" else PoseSource.GPS
    kwargs = {"pose_source": pose_source, "start_driver": False}
    if args.connection:
        kwargs["connection_string"] = args.connection

    if args.drone == "mavros":
        return MavrosConfig(**kwargs)
    if args.drone == "mavlink":
        return MavlinkConfig(**kwargs)
    if args.drone == "px4":
        return Px4MavrosConfig(**kwargs)
    if args.drone == "px4_mavlink":
        return Px4MavlinkConfig(**kwargs)
    if args.drone == "px4_dds":
        return Px4DdsConfig(pose_source=pose_source, start_driver=False)
    raise ValueError(f"Unsupported --drone for this mission: {args.drone}")


def build_camera_config(camera_type: str):
    """Return (config, source_key) for ImageHandler."""
    params = _CAMERA_PARAMS.get(camera_type)
    if params is None:
        # A raw ROS topic (starts with '/') or a video file path is passed
        # straight through to ImageHandler / CameraFactory.
        return None, camera_type
    return ConfigBuilder.build(camera_type, params), camera_type


def best_candidate(result, target_classes: Optional[Set[str]]):
    """Highest-confidence detection matching `target_classes` (or None)."""
    if not result:
        return None
    candidates = [d for d in result if target_classes is None or d.class_name in target_classes]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def matches_target(result, target_classes: Optional[Set[str]]) -> bool:
    return best_candidate(result, target_classes) is not None


def report_detection(detector: Detector, state: DetectionState, target_classes, snapshot_path: str) -> None:
    with state.lock:
        result = state.result
        frame = state.frame

    best = best_candidate(result, target_classes)
    if best is None:
        return
    log.info(
        "BASE FOUND -> class=%s conf=%.2f center=%s bbox=%s",
        best.class_name,
        best.confidence,
        best.center,
        best.bbox,
    )

    if frame is not None:
        annotated = detector.draw_detections(frame, result)
        cv2.imwrite(snapshot_path, annotated)
        log.info("Saved detection snapshot to %s", snapshot_path)


def center_and_land(drone, state: DetectionState, target_classes, args: argparse.Namespace) -> None:
    """Visually servo onto the base, descending once centered, then land.

    Uses two Nectar `PIDController`s -- Nectar's documented, framework-generic
    control-loop primitive -- to drive the base's normalized pixel offset
    from the image center to zero, commanding BODY-frame velocities via
    `move_velocity`. Descends only while centered; hands off to `land()` once
    low enough or if the target is lost for too long.
    """
    pid_lateral = PIDController(
        kp=args.kp, ki=args.ki, kd=args.kd, setpoint=0.0, output_limits=(-args.max_correction, args.max_correction)
    )
    pid_forward = PIDController(
        kp=args.kp, ki=args.ki, kd=args.kd, setpoint=0.0, output_limits=(-args.max_correction, args.max_correction)
    )

    lost_frames = 0
    start = time.time()
    while time.time() - start < args.landing_timeout:
        with state.lock:
            result = state.result
            frame = state.frame

        best = best_candidate(result, target_classes)
        if best is None or frame is None:
            lost_frames += 1
            if lost_frames > args.max_lost_frames:
                log.warning("Target lost during final approach; landing at current position.")
                return
            drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, duration=None, reference=MoveReference.BODY)
            drone.delay(0.1)
            continue
        lost_frames = 0

        h, w = frame.shape[:2]
        cx, cy = best.center
        err_x = (cx - w / 2.0) / (w / 2.0)  # [-1, 1], + = target to the right
        err_y = (cy - h / 2.0) / (h / 2.0)  # [-1, 1], + = target below center

        vy = args.lateral_sign * pid_lateral.update(err_x)
        vx = args.forward_sign * pid_forward.update(err_y)

        centered = abs(err_x) < args.align_tolerance and abs(err_y) < args.align_tolerance
        vz = -args.descent_speed if centered else 0.0  # move_velocity: vz up(+)/down(-)

        altitude = drone.get_altitude()
        if altitude is not None and altitude <= args.land_altitude:
            log.info("Centered and low (%.2fm); handing off to land().", altitude)
            return

        drone.move_velocity(vx=vx, vy=vy, vz=vz, duration=None, reference=MoveReference.BODY)
        drone.delay(0.1)

    log.warning("Landing phase timed out after %.0fs; landing at current position.", args.landing_timeout)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Fly forward and identify the base (Nectar SDK)")
    parser.add_argument(
        "--drone",
        choices=["mavlink", "mavros", "px4", "px4_mavlink", "px4_dds"],
        default="mavlink",
        help="Nectar drone backend to use.",
    )
    parser.add_argument("--env", choices=["outdoor", "indoor"], default="outdoor")
    parser.add_argument("--connection", default=None, help="Connection string override (mavlink) or fcu_url (mavros).")
    parser.add_argument("--camera-type", default="webcam", help="webcam | imx219 | ros | <video file path> | <ROS topic>")
    parser.add_argument("--model", default="models/best.pt", help="Path to the YOLO best.pt weights.")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum detection confidence.")
    parser.add_argument(
        "--classes",
        nargs="*",
        default=None,
        help="Class names that count as 'base'. Default: accept any detected class.",
    )
    parser.add_argument("--confirm-frames", type=int, default=3, help="Consecutive positive frames required to confirm.")
    parser.add_argument("--height", type=float, default=2.0, help="Takeoff altitude (m).")
    parser.add_argument("--speed", type=float, default=0.3, help="Forward speed (m/s).")
    parser.add_argument("--step-duration", type=float, default=0.5, help="Seconds flown forward per control step.")
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

    nectar.init()

    detector = Detector(args.model)
    detector.load()
    log.info("Loaded model %s (%s) classes=%s", args.model, detector.framework, detector.class_names)

    state = DetectionState()

    def on_frame(frame) -> None:
        if frame is None:
            return
        result = detector.detect(frame, conf=args.conf)
        with state.lock:
            state.frame = frame
            state.result = result
            if matches_target(result, target_classes):
                state.consecutive_hits += 1
            else:
                state.consecutive_hits = 0

    cam_config, cam_source = build_camera_config(args.camera_type)
    camera = ImageHandler(
        image_source=cam_source,
        config=cam_config,
        image_processing_callback=on_frame,
        poll_interval=0.05,
    )
    camera.run()

    drone = DroneFactory.create(args.drone, build_drone_config(args))
    found = False
    try:
        if not drone.connect():
            log.error("Failed to connect to the vehicle")
            return
        if not drone.takeoff(altitude=args.height):
            log.error("Takeoff failed")
            return
        drone.delay(2.0)

        log.info("Flying forward at %.2f m/s, searching for the base...", args.speed)
        start = time.time()
        while time.time() - start < args.search_timeout:
            with state.lock:
                hits = state.consecutive_hits
            if hits >= args.confirm_frames:
                found = True
                break
            drone.move_velocity(vx=args.speed, duration=args.step_duration)

        if found:
            drone.move_velocity(vx=0.0, vy=0.0, duration=0.5)  # stop and hover
            report_detection(detector, state, target_classes, args.snapshot)
            log.info("Centering on the base and descending to land on top of it...")
            center_and_land(drone, state, target_classes, args)
        else:
            log.warning("Base not identified within %.0fs; landing at current position.", args.search_timeout)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        camera.cleanup()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
