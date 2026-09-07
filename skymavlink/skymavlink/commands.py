"""Companion-computer commands: servo, GCS text, parameters."""

from pymavlink import mavutil

from .core import _Core


class _Commands(_Core):
    """Mixin: commands a mission sends that are not flight or motion."""

    def set_servo(self, channel: int, pwm: int) -> None:
        """Drive a servo output directly.

        The output's `SERVOx_FUNCTION` must be 0 (Disabled) for the FC to hand
        the channel over to MAVLink.

        Args:
            channel: servo output number, 1-based.
            pwm:     pulse width in microseconds, 800–2200.
        """
        if channel < 1:
            raise ValueError(f'servo channel must be >= 1, got {channel}')
        if not 800 <= pwm <= 2200:
            raise ValueError(f'servo PWM must be 800–2200 µs, got {pwm}')
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            float(channel), float(pwm), 0, 0, 0, 0, 0)
        self._log.info(f'set_servo ch={channel} pwm={pwm}')

    def send_statustext(self, text: str, severity: int = 6) -> None:
        """Show a message on the ground station. Truncated to 50 bytes."""
        if not 0 <= severity <= 7:
            raise ValueError(f'severity must be 0–7, got {severity}')
        self._conn.mav.statustext_send(severity, text.encode('utf-8')[:50])

    def set_parameter(self, name: str, value: float) -> None:
        """Set a numeric FC parameter. Fire-and-forget — no ack is awaited."""
        encoded = name.encode('ascii')
        if not 1 <= len(encoded) <= 16:
            raise ValueError(f'parameter name must be 1–16 ASCII bytes: {name!r}')
        self._conn.mav.param_set_send(
            self._conn.target_system, self._conn.target_component,
            encoded, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self._log.info(f'set_parameter {name}={value}')
