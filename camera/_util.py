"""Shared helpers for the camera/ mission scripts."""

import logging
import math
import os
import time

import cv2

log = logging.getLogger("camera")

ENDPOINT_SITL = "tcp:127.0.0.1:5760"


def current_heading_deg(drone) -> float:
    """Current yaw in degrees CW from North, or 0.0 if attitude hasn't arrived yet."""
    att = drone.get_attitude()
    return math.degrees(att[2]) if att is not None else 0.0


def wait_until_arrived(drone, target_n, target_e, tolerance=0.2, timeout=30.0, poll_period=0.2, on_tick=None):
    """Poll get_pose_local() until within `tolerance` meters of the target.

    Calls on_tick() once per poll, if given -- used here to take a photo on a
    fixed cadence while a leg is in flight, without a second thread.
    """
    elapsed = 0.0
    while elapsed < timeout:
        drone.sleep(poll_period)
        elapsed += poll_period
        if on_tick is not None:
            on_tick()
        pose = drone.get_pose_local()
        if pose is None:
            continue
        n, e, _d = pose
        if math.hypot(target_n - n, target_e - e) <= tolerance:
            return True
    return False


class PhotoSaver:
    """Saves camera frames to disk on a fixed cadence, numbered sequentially."""

    def __init__(self, cap, out_dir: str, period: float = 1.0, prefix: str = "foto") -> None:
        self.cap = cap
        self.out_dir = out_dir
        self.period = period
        self.prefix = prefix
        self._count = 0
        self._last_saved = 0.0
        os.makedirs(out_dir, exist_ok=True)

    @property
    def count(self) -> int:
        return self._count

    def maybe_capture(self, force: bool = False):
        """Grab and save a frame if `period` has elapsed since the last one.

        Returns the saved path, or None if it's not time yet / the read failed.
        """
        now = time.time()
        if not force and (now - self._last_saved) < self.period:
            return None
        ok, frame = self.cap.read()
        if not ok:
            return None
        self._last_saved = now
        self._count += 1
        path = os.path.join(self.out_dir, f"{self.prefix}_{self._count:03d}.jpg")
        cv2.imwrite(path, frame)
        log.info("Saved %s", path)
        return path
