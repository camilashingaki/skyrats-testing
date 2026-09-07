"""Motion commands: local pose, body pose, body velocity, global pose."""

import math

from pymavlink import mavutil

from .core import _Core


# type_mask constants for SET_POSITION_TARGET_LOCAL_NED
# Bit = 1 means "ignore this field"
_MASK_POS_ONLY   = 0x0FF8  # use position only
_MASK_POS_YAW    = 0x09F8  # use position + yaw
_MASK_VEL        = 0x0DC7  # use velocity only — see set_body_velocity, not used
_MASK_VEL_YAW_RT = 0x05C7  # use velocity + yaw_rate


class _Motion(_Core):
    """Mixin: motion setpoints in NED (world) and FRD (body) frames.

    Coordinate conventions
    ----------------------
    NED world frame:  North = +X, East = +Y, Down = +Z
      - Altitude above ground = negative Down (e.g. 1.5 m high → down = -1.5)
      - Yaw: clockwise from North in radians (0 = North, π/2 = East)

    FRD body frame:   Forward = +X (nose), Right = +Y, Down = +Z
      - Yaw rate: clockwise positive (degrees/s)

    Velocity commands are stored and re-sent at 20 Hz by _tick() while the
    mission thread is inside sleep(). End a leg with set_body_velocity(0, 0, 0);
    a position command, land() or kill() also cancels it.
    """

    def set_local_pose(
        self,
        north: float,
        east: float,
        down: float,
        yaw_deg: float = None,
    ) -> None:
        """Go to an absolute NED position.

        Args:
            north:   metres from EKF origin toward North (+) or South (−).
            east:    metres from EKF origin toward East (+) or West (−).
            down:    metres downward. Negative value = above origin
                     (e.g. down=-1.5 to fly at 1.5 m altitude).
            yaw_deg: target heading, degrees CW from North. None = no yaw change.
        """
        self._vel_cmd = None
        tmask = _MASK_POS_YAW if yaw_deg is not None else _MASK_POS_ONLY
        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else 0.0
        self._conn.mav.set_position_target_local_ned_send(
            0,
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, tmask,
            float(north), float(east), float(down),
            0, 0, 0,
            0, 0, 0,
            yaw_rad, 0)
        yaw_str = f' yaw={yaw_deg:.1f}°' if yaw_deg is not None else ''
        self._log.info(
            f'set_local_pose N={north:.2f} E={east:.2f} D={down:.2f}{yaw_str}')

    def set_body_pose(
        self,
        fwd: float,
        right: float,
        down: float,
        yaw_deg: float = None,
    ) -> None:
        """Move by a FRD offset relative to the current NED position.

        The offset is rotated by the current heading into NED before sending
        as an absolute position setpoint.

        Args:
            fwd:     metres forward (toward nose).
            right:   metres to the right.
            down:    metres downward (negative = up).
            yaw_deg: target heading, degrees CW from North. None = no yaw change.
        """
        if self._pos_ned is None:
            self._log.warning('set_body_pose: no position — call wait_for_position() first')
            return
        yaw = self._yaw_ned          # current heading, rad CW from North
        c, s = math.cos(yaw), math.sin(yaw)
        pn, pe, pd = self._pos_ned
        # Heading α: forward=(cos α, sin α) and right=(-sin α, cos α) in (N, E)
        target_n = pn + fwd * c + right * (-s)
        target_e = pe + fwd * s + right * c
        target_d = pd + down
        self.set_local_pose(target_n, target_e, target_d, yaw_deg)

    def set_body_velocity(
        self,
        fwd: float,
        right: float,
        down: float,
        yaw_rate_dps: float = 0.0,
    ) -> None:
        """Set a continuous body-frame FRD velocity setpoint.

        The setpoint is re-sent at 20 Hz by the tick loop until another motion
        command replaces it.

        Args:
            fwd:          m/s forward (positive = toward nose).
            right:        m/s rightward (positive = right wing).
            down:         m/s downward (positive = descend).
            yaw_rate_dps: degrees/s clockwise. 0 = hold the current heading.
        """
        yr = math.radians(yaw_rate_dps)
        # Always send the yaw-rate field, even when it is zero. With _MASK_VEL the
        # yaw_rate field is flagged "ignore", and ArduPilot then falls back to
        # WP_YAW_BEHAVIOR (default: face the next waypoint), which turns the nose
        # into the direction of travel — measured at 80° of drift over a 1 m square
        # flown at 0.8 m/s. Sending an explicit zero rate holds the heading.
        tmask = _MASK_VEL_YAW_RT
        # MAV_FRAME_BODY_NED: vx=fwd, vy=right, vz=down — matches FRD directly
        self._vel_cmd = (
            mavutil.mavlink.MAV_FRAME_BODY_NED, tmask,
            float(fwd), float(right), float(down), yr)

    def set_global_pose(
        self,
        lat: float,
        lon: float,
        alt_m_agl: float,
    ) -> None:
        """Go to a GPS coordinate (requires GPS-capable EKF).

        Args:
            lat:       latitude in decimal degrees.
            lon:       longitude in decimal degrees.
            alt_m_agl: altitude in metres above the home point.
        """
        self._vel_cmd = None
        self._conn.mav.set_position_target_global_int_send(
            0,
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0FF8,             # position only
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt_m_agl),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0)
        self._log.info(
            f'set_global_pose lat={lat:.6f} lon={lon:.6f} alt={alt_m_agl:.1f}m')
