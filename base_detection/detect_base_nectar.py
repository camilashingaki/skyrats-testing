#!/usr/bin/env python3
"""
Base identification mission using the Nectar SDK (Black-Bee-Drones/nectar-sdk).

The drone takes off, flies forward at a constant velocity, and continuously
runs a YOLO model (nectar.ai.detection.Detector, loaded from a `best.pt`
checkpoint) on the live camera feed. As soon as the base is confirmed for a
few consecutive frames, the drone stops, hovers, logs the detection and saves
an annotated snapshot, then lands.

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
    PoseSource,
    Px4DdsConfig,
    Px4MavlinkConfig,
    Px4MavrosConfig,
)
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


def matches_target(result, target_classes: Optional[Set[str]]) -> bool:
    if not result:
        return False
    if target_classes is None:
        return len(result) > 0
    return any(det.class_name in target_classes for det in result)


def report_detection(detector: Detector, state: DetectionState, target_classes, snapshot_path: str) -> None:
    with state.lock:
        result = state.result
        frame = state.frame

    candidates = [d for d in result if target_classes is None or d.class_name in target_classes]
    if not candidates:
        return
    best = max(candidates, key=lambda d: d.confidence)
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
