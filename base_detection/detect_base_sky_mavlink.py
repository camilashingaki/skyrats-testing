#!/usr/bin/env python3
"""
Base identification mission using SkyRats/sky_mavlink (SkyMAVLink).

The drone arms, takes off, flies forward at a constant body-frame velocity,
and continuously runs a YOLO model (ultralytics, loaded from a `best.pt`
checkpoint) on the live camera feed. As soon as the base is confirmed for a
few consecutive frames, the drone brakes, hovers, logs the detection and
saves an annotated snapshot, then lands.

SkyMAVLink has no background thread: its message loop only advances inside
blocking calls (`sleep()`, `takeoff()`, `arm()`, ...), so the search loop
below calls `drone.sleep(step)` on every iteration -- this both services the
MAVLink link and keeps re-sending the active `set_body_velocity` setpoint.
Camera capture + YOLO inference run on a separate Python thread, independent
of the MAVLink link.

Requirements:
    pip install -r requirements.txt
    pip install -e /path/to/sky_mavlink   # SkyMAVLink itself (not on PyPI)

Usage:
    python detect_base_sky_mavlink.py --connection tcp:127.0.0.1:5760 --model models/best.pt
    python detect_base_sky_mavlink.py --connection serial:/dev/ttyACM0:115200 --model models/best.pt --classes base
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


def matches_target(result, class_names, target_classes: Optional[Set[str]]) -> bool:
    if result is None or result.boxes is None or len(result.boxes) == 0:
        return False
    if target_classes is None:
        return True
    for cls_id in result.boxes.cls.tolist():
        if class_names[int(cls_id)] in target_classes:
            return True
    return False


def report_detection(detector: Detector, state: DetectionState, target_classes, snapshot_path: str) -> None:
    with state.lock:
        result = state.result
        frame = state.frame

    class_names = detector.class_names
    best_idx, best_conf, best_name = None, -1.0, None
    for i, (cls_id, conf) in enumerate(zip(result.boxes.cls.tolist(), result.boxes.conf.tolist())):
        name = class_names[int(cls_id)]
        if target_classes is not None and name not in target_classes:
            continue
        if conf > best_conf:
            best_idx, best_conf, best_name = i, conf, name

    if best_idx is None:
        return

    xyxy = result.boxes.xyxy[best_idx].tolist()
    log.info("BASE FOUND -> class=%s conf=%.2f bbox=%s", best_name, best_conf, xyxy)

    if frame is not None:
        annotated = detector.draw(result)
        cv2.imwrite(snapshot_path, annotated)
        log.info("Saved detection snapshot to %s", snapshot_path)


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
        default="tcp:127.0.0.1:5760",
        help="pymavlink endpoint, e.g. tcp:127.0.0.1:5760 (SITL), udpout:HOST:PORT, or serial:/dev/ttyACM0:115200.",
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
