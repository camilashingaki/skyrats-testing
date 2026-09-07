"""Flight operations: mode, arm, kill, takeoff, land, RTL."""

import time

from pymavlink import mavutil

from .core import _Core


_ACM_MODE = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
    'GUIDED': 4,    'LOITER': 5, 'RTL': 6,     'CIRCLE': 7,
    'LAND': 9,      'SPORT': 13, 'POSHOLD': 16, 'BRAKE': 17,
    'SMART_RTL': 21,
}


class _Flight(_Core):
    """Mixin: flight operations (mode, arm, kill, takeoff, land, RTL).

    All methods that wait for FC confirmation loop on _tick() and raise
    TimeoutError if the FC does not confirm within `timeout` seconds.
    """

    def set_mode(self, mode: str, timeout: float = 15.0) -> None:
        """Switch to a named ArduCopter mode and wait for confirmation."""
        name = mode.upper()
        if name not in _ACM_MODE:
            raise ValueError(
                f'unknown mode {mode!r} — valid: {list(_ACM_MODE)}')
        self._log.info(f'set mode {name}')
        self._retry_until(
            lambda: self._send_mode(name),
            lambda: self._mode == _ACM_MODE[name],
            timeout,
            f'mode {name}',
        )

    def arm(self, timeout: float = 15.0) -> None:
        """Arm the motors and wait for the FC to confirm.

        On timeout the error carries the FC's recent STATUSTEXT messages, which
        name the pre-arm check that refused.
        """
        self._log.info('arming')
        try:
            self._retry_until(
                lambda: self._conn.mav.command_long_send(
                    self._conn.target_system, self._conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0),
                lambda: self._armed,
                timeout,
                'arm',
            )
        except TimeoutError as exc:
            recent = self.recent_statustexts(max(15.0, timeout))[-4:]
            detail = '; '.join(text for _t, _sev, text in recent)
            suffix = f' — FC said: {detail}' if detail else ''
            raise TimeoutError(
                f'arm not confirmed within {timeout:.0f}s{suffix}') from exc

    def kill(self) -> None:
        """Force-disarm immediately, even in flight. Drone will fall."""
        self._log.warning('KILL — force disarming')
        self._vel_cmd = None
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0,      # param1 = 0 → disarm
            21196,  # param2 = 21196 → force (ArduPilot magic)
            0, 0, 0, 0, 0)

    def takeoff(self, altitude: float = None, timeout: float = 30.0) -> None:
        """Command takeoff and block until 95% of target altitude is reached.

        Args:
            altitude: target altitude in metres AGL. Defaults to takeoff_altitude.
            timeout:  maximum wait time in seconds.
        """
        alt = self.takeoff_altitude if altitude is None else float(altitude)
        self._log.info(f'takeoff {alt:.1f} m')
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt)
        end = time.time() + timeout
        while time.time() < end:
            self._tick()
            # NED down is negative above ground: reached when down ≤ -(0.95 × alt)
            if self._pos_ned and self._pos_ned[2] <= -(0.95 * alt):
                break
        reached = -self._pos_ned[2] if self._pos_ned else 0.0
        self._log.info(f'reached {reached:.2f} m')

    def land(self, speed_ms: float = None, timeout: float = 60.0) -> None:
        """Switch to LAND mode and wait until disarmed.

        Args:
            speed_ms: optional descent speed in m/s (sets LAND_SPEED FC param).
            timeout:  maximum wait time in seconds.
        """
        self._log.info('landing')
        if speed_ms is not None:
            self._conn.mav.param_set_send(
                self._conn.target_system, self._conn.target_component,
                b'LAND_SPEED',      # pymavlink requires bytes, not str
                speed_ms * 100.0,   # ArduPilot LAND_SPEED is in cm/s
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.sleep(0.5)         # let FC apply param before mode switch
        self._vel_cmd = None
        self._retry_until(
            lambda: self._send_mode('LAND'),
            lambda: not self._armed,
            timeout,
            'land',
            period=2.0,
        )
        self._log.info('landed and disarmed')

    def rtl(self) -> None:
        """Switch to Return-to-Launch mode (non-blocking)."""
        self._log.info('RTL')
        self._send_mode('RTL')

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send_mode(self, name: str) -> None:
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            _ACM_MODE[name], 0, 0, 0, 0, 0)
