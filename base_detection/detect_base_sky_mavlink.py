#!/usr/bin/env python3
"""
Base identification mission using MAVLink directly (pymavlink), in the same
spirit as SkyRats/sky_mavlink.

NOTE: SkyRats/sky_mavlink is a private repository this session could not read
(no access granted), so its exact class/method names could not be confirmed.
This script talks MAVLink directly through `pymavlink.mavutil`, which is the
library sky_mavlink itself wraps, and isolates every vehicle command inside
the `MavlinkDrone` class below. If sky_mavlink exposes a higher-level API
(e.g. a `Drone`/`Vehicle` class with `arm()`, `takeoff()`, `send_velocity()`),
swap the body of `MavlinkDrone`'s methods for calls into that class -- the
mission logic in `main()` does not need to change.

The drone arms, takes off, flies forward at a constant body-frame velocity,
and continuously runs a YOLO model (ultralytics, loaded from a `best.pt`
checkpoint) on the live camera feed. As soon as the base is confirmed for a
few consecutive frames, the drone stops, hovers, logs the detection and saves
an annotated snapshot, then lands.

Requirements:
    pip install pymavlink opencv-python ultralytics

Usage:
    python detect_base_sky_mavlink.py --connection udp:127.0.0.1:14550 --model models/best.pt
    python detect_base_sky_mavlink.py --connection /dev/ttyACM0 --model models/best.pt --classes base
"""

import argparse
import logging
import threading
import time
from typing import Optional, Set

import cv2
from pymavlink import mavutil
from ultralytics import YOLO

log = logging.getLogger("base_detection_sky_mavlink")

_M = mavutil.mavlink

# type_mask for SET_POSITION_TARGET_LOCAL_NED: use velocity (vx,vy,vz) only,
# ignore position, acceleration, and yaw/yaw-rate fields.
_VELOCITY_ONLY_MASK = 0b0000111111000111


class MavlinkDrone:
    """Thin wrapper around a pymavlink connection for basic guided flight."""

    def __init__(self, connection_string: str, baud: Optional[int] = None) -> None:
        self.connection_string = connection_string
        self.baud = baud
        self.master: Optional[mavutil.mavfile] = None

    def connect(self, heartbeat_timeout: float = 30.0) -> bool:
        kwargs = {"baud": self.baud} if self.baud else {}
        self.master = mavutil.mavlink_connection(self.connection_string, **kwargs)
        log.info("Waiting for heartbeat on %s ...", self.connection_string)
        msg = self.master.wait_heartbeat(timeout=heartbeat_timeout)
        if msg is None:
            log.error("No heartbeat received within %.0fs", heartbeat_timeout)
            return False
        log.info(
            "Heartbeat received (system %d, component %d)",
            self.master.target_system,
            self.master.target_component,
        )
        return True

    def set_mode(self, mode: str) -> bool:
        mapping = self.master.mode_mapping() or {}
        mode_id = mapping.get(mode)
        if mode_id is None:
            log.error("Unknown flight mode '%s'. Available: %s", mode, sorted(mapping))
            return False
        self.master.mav.set_mode_send(
            self.master.target_system, _M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
        )
        return True

    def arm(self, timeout: float = 10.0) -> bool:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            _M.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        try:
            self.master.motors_armed_wait(timeout=timeout)
        except TypeError:
            # Older pymavlink versions don't accept a timeout kwarg here.
            self.master.motors_armed_wait()
        return bool(self.master.motors_armed())

    def disarm(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            _M.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    def get_relative_altitude(self, timeout: float = 1.0) -> Optional[float]:
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
        if msg is None:
            return None
        return msg.relative_alt / 1000.0

    def takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        if not self.set_mode("GUIDED"):
            return False
        if not self.arm():
            log.error("Failed to arm")
            return False

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            _M.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude,
        )

        start = time.time()
        while time.time() - start < timeout:
            alt = self.get_relative_altitude(timeout=1.0)
            if alt is not None:
                log.info("Altitude: %.2fm / %.2fm", alt, altitude)
                if alt >= altitude * 0.95:
                    return True
        log.error("Timed out waiting to reach takeoff altitude")
        return False

    def send_velocity(self, vx: float, vy: float, vz: float = 0.0) -> None:
        """Send one body-frame velocity setpoint (m/s, FRD: x=fwd, y=right, z=down)."""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            _M.MAV_FRAME_BODY_OFFSET_NED,
            _VELOCITY_ONLY_MASK,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )

    def move_forward(self, speed: float, duration: float, rate_hz: float = 10.0) -> None:
        """Fly forward at `speed` m/s for `duration` seconds, then stop."""
        period = 1.0 / rate_hz
        steps = max(1, int(duration / period))
        for _ in range(steps):
            self.send_velocity(speed, 0.0, 0.0)
            time.sleep(period)
        self.send_velocity(0.0, 0.0, 0.0)

    def hover(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0)

    def land(self) -> None:
        self.set_mode("LAND")

    def close(self) -> None:
        if self.master is not None:
            self.master.close()


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
    """Shared, lock-protected view of the latest detector output."""

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
    parser = argparse.ArgumentParser(description="Fly forward and identify the base (MAVLink)")
    parser.add_argument("--connection", default="udp:127.0.0.1:14550", help="MAVLink connection string (udp:host:port, tcp:host:port, or a serial path).")
    parser.add_argument("--baud", type=int, default=None, help="Baud rate for serial connections.")
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
    parser.add_argument("--height", type=float, default=2.0, help="Takeoff altitude (m).")
    parser.add_argument("--speed", type=float, default=0.3, help="Forward speed (m/s).")
    parser.add_argument("--step-duration", type=float, default=0.5, help="Seconds flown forward per control step.")
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

    drone = MavlinkDrone(args.connection, baud=args.baud)
    found = False
    try:
        if not drone.connect():
            log.error("Failed to connect to the vehicle")
            return
        if not drone.takeoff(args.height):
            log.error("Takeoff failed")
            return

        log.info("Flying forward at %.2f m/s, searching for the base...", args.speed)
        start = time.time()
        while time.time() - start < args.search_timeout:
            with state.lock:
                hits = state.consecutive_hits
            if hits >= args.confirm_frames:
                found = True
                break
            drone.move_forward(args.speed, args.step_duration)

        if found:
            drone.hover()
            report_detection(detector, state, target_classes, args.snapshot)
        else:
            log.warning("Base not identified within %.0fs; landing at current position.", args.search_timeout)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        stop_event.set()
        cam_thread.join(timeout=2.0)
        cap.release()
        drone.close()


if __name__ == "__main__":
    main()
