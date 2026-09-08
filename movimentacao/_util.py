"""Shared helper for the movimentacao/ mission scripts."""

import logging
import math

log = logging.getLogger("movimentacao")

ENDPOINT_SITL = "tcp:127.0.0.1:5760"


def wait_until_arrived(drone, target_n, target_e, tolerance=0.2, timeout=30.0, poll_period=0.2):
    """Poll get_pose_local() until within `tolerance` meters of (target_n, target_e).

    Position commands (set_local_pose/set_body_pose) return immediately, so a
    mission that needs to know when the drone actually arrived -- to fly an
    exact distance, or to start the next leg of a square -- has to close the
    loop itself on get_pose_local(), never on speed * time.
    """
    elapsed = 0.0
    while elapsed < timeout:
        drone.sleep(poll_period)
        elapsed += poll_period
        pose = drone.get_pose_local()
        if pose is None:
            continue
        n, e, _d = pose
        if math.hypot(target_n - n, target_e - e) <= tolerance:
            return True
    return False


def current_heading_deg(drone) -> float:
    """Current yaw in degrees CW from North, or 0.0 if attitude hasn't arrived yet."""
    att = drone.get_attitude()
    return math.degrees(att[2]) if att is not None else 0.0
