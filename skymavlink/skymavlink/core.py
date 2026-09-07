"""Core: MAVLink connection, message loop, and shared state."""

import logging
import time
from collections import deque

from pymavlink import mavutil


def _parse_endpoint(endpoint: str) -> tuple[str, dict]:
    """Split a `serial:/dev/device:baud` endpoint into pymavlink arguments.

    pymavlink takes a bare device path and a separate `baud` keyword; it has no
    `serial:` prefix. Given one it falls through to the UDP parser and raises
    "UDP ports must be specified as host:port". Every other endpoint form is
    passed through untouched.
    """
    if not endpoint.startswith('serial:'):
        return endpoint, {}
    spec = endpoint[len('serial:'):]
    device, _, baud_str = spec.rpartition(':')
    if not device or not baud_str.isdigit() or int(baud_str) <= 0:
        raise ValueError(
            f'bad serial endpoint {endpoint!r} — expected '
            'serial:/dev/device:baud, e.g. serial:/dev/ttyACM0:115200')
    return device, {'baud': int(baud_str)}


class _Core:
    """Base class: opens the MAVLink connection and runs the synchronous tick loop.

    All blocking methods loop on _tick(), which drains incoming messages,
    sends a 1 Hz heartbeat so the router in front of the FC keeps routing replies
    back to us, and re-sends the active velocity setpoint at ~20 Hz.
    """

    _HB_PERIOD = 1.0  # heartbeat interval (s) — keeps the router routing to us

    # Our MAVLink identity. sysid 1 matches the vehicle; the component id must NOT
    # be 191 (MAV_COMP_ID_ONBOARD_COMPUTER) — MAVROS transmits as (1, 191) by
    # default, and on a shared link two endpoints with the same address make
    # targeted FC replies (COMMAND_ACK) ambiguous to the MAVROS router.
    _SRC_SYSTEM = 1
    _SRC_COMPONENT = 192  # MAV_COMP_ID_ONBOARD_COMPUTER2

    def __init__(
        self,
        endpoint: str,
        takeoff_altitude: float = 1.5,
    ):
        # `endpoint` is required on purpose. The transport belongs to whoever
        # instantiates the object — SITL, mavp2p and direct serial are all valid
        # and the library has no way to know which one is in front of it. A
        # default here silently flies (or silently fails to fly) the wrong link.
        self.takeoff_altitude = takeoff_altitude

        if not logging.root.handlers:
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s [skymavlink] %(levelname)s: %(message)s',
                datefmt='%H:%M:%S',
            )
        self._log = logging.getLogger('skymavlink')

        # FC state — updated from incoming MAVLink messages
        self._connected: bool = False
        self._armed: bool = False
        self._mode: int = -1

        # Position in NED metres: (north, east, down)
        # Negative down = positive altitude above takeoff point.
        self._pos_ned: tuple[float, float, float] | None = None

        # Global position: (lat_deg, lon_deg, alt_m_agl) — altitude relative to
        # home, matching the frame set_global_pose() commands in.
        self._pos_global: tuple[float, float, float] | None = None

        # Attitude from ATTITUDE message
        self._yaw_ned: float = 0.0                               # rad, CW from North
        self._att_ned: tuple[float, float, float] | None = None  # (roll, pitch, yaw)

        # GPS receiver health: (fix_type, satellites). Not derivable from
        # _pos_global — GLOBAL_POSITION_INT is the EKF's fused output and keeps
        # arriving after the GPS degrades.
        self._gps_fix: tuple[int, int | None] | None = None

        # Recent STATUSTEXT from the FC: (time, severity, text). Pre-arm
        # failures arrive here and nowhere else.
        self._statustexts: deque[tuple[float, int, str]] = deque(maxlen=50)

        # Active velocity command: (frame, type_mask, vx, vy, vz, yaw_rate)
        # None means no velocity is being sent.
        self._vel_cmd: tuple | None = None

        self._last_hb: float = 0.0

        device, opts = _parse_endpoint(endpoint)
        self._conn = mavutil.mavlink_connection(
            device,
            source_system=self._SRC_SYSTEM,
            source_component=self._SRC_COMPONENT,
            **opts)
        self._conn.target_system = 1
        self._conn.target_component = 1

    # ── Incoming message processing ───────────────────────────────────────────

    def _drain(self) -> None:
        """Non-blocking: process all pending MAVLink messages from the FC."""
        while True:
            msg = self._conn.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != self._conn.target_system:
                continue
            mtype = msg.get_type()

            if mtype == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                self._connected = True
                self._armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._mode = msg.custom_mode

            elif mtype == 'LOCAL_POSITION_NED':
                # ArduPilot reports NED directly: x=North, y=East, z=Down
                self._pos_ned = (msg.x, msg.y, msg.z)

            elif mtype == 'GLOBAL_POSITION_INT':
                # lat/lon are degE7, relative_alt is mm above the home point
                self._pos_global = (
                    msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)

            elif mtype == 'ATTITUDE':
                # NED yaw: 0 = North, positive CW when viewed from above
                self._yaw_ned = msg.yaw
                self._att_ned = (msg.roll, msg.pitch, msg.yaw)

            elif mtype == 'GPS_RAW_INT':
                sats = int(msg.satellites_visible)
                # 255 is the "unknown" sentinel, not a count
                self._gps_fix = (
                    int(msg.fix_type), None if sats == 255 else sats)

            elif mtype == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', 'replace')
                self._statustexts.append(
                    (time.time(), int(msg.severity), text.rstrip('\0')))

    # ── Tick: one ~50 ms cycle ────────────────────────────────────────────────

    def _tick(self) -> None:
        """Drain messages, send heartbeat if due, re-send active velocity."""
        self._drain()

        now = time.time()
        if now - self._last_hb >= self._HB_PERIOD:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
            self._last_hb = now

        if self._vel_cmd is not None:
            frame, tmask, vx, vy, vz, yr = self._vel_cmd
            self._conn.mav.set_position_target_local_ned_send(
                0,
                self._conn.target_system, self._conn.target_component,
                frame, tmask,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, yr)

        time.sleep(0.05)

    # ── Blocking helpers ──────────────────────────────────────────────────────

    def _wait_until(self, predicate, timeout: float, label: str) -> None:
        """Loop on _tick() until predicate() is True or timeout expires."""
        end = time.time() + timeout
        while time.time() < end:
            self._tick()
            if predicate():
                return
        raise TimeoutError(f'{label} timed out after {timeout:.0f}s')

    def _retry_until(
        self,
        send_fn,
        predicate,
        timeout: float,
        label: str,
        period: float = 1.0,
    ) -> None:
        """Resend a command every `period` seconds until predicate() is True."""
        end = time.time() + timeout
        last_send = 0.0
        while time.time() < end:
            self._tick()
            if predicate():
                self._log.info(f'{label} confirmed')
                return
            now = time.time()
            if now - last_send >= period:
                send_fn()
                last_send = now
        raise TimeoutError(f'{label} not confirmed within {timeout:.0f}s')

    # ── Public: state ─────────────────────────────────────────────────────────

    def get_logger(self) -> logging.Logger:
        return self._log

    def get_pose_local(self) -> tuple[float, float, float] | None:
        """Current NED position in metres: (north, east, down).

        Down is negative above the takeoff point (e.g. -1.5 at 1.5 m altitude).
        Returns None until the first LOCAL_POSITION_NED message is received.
        """
        return self._pos_ned

    def get_pose_global(self) -> tuple[float, float, float] | None:
        """Current global position: (latitude, longitude, altitude).

        Latitude and longitude are decimal degrees; altitude is metres above the
        home point — the same frame set_global_pose() takes, so the two round-trip.
        Returns None until the first GLOBAL_POSITION_INT message is received; the
        FC only emits it once the EKF has a global origin, so it stays None on a
        vision-only (no GPS) setup.
        """
        return self._pos_global

    def get_attitude(self) -> tuple[float, float, float] | None:
        """Current attitude in radians (NED): (roll, pitch, yaw).

        Yaw is clockwise from North: 0 = North, π/2 = East.
        Returns None until the first ATTITUDE message is received.
        """
        return self._att_ned

    def get_gps_fix(self) -> tuple[int, int | None] | None:
        """GPS receiver health: (fix_type, satellites), or None before the first
        GPS_RAW_INT.

        fix_type is the MAVLink GPS_FIX_TYPE scale — 0 no GPS, 1 no fix,
        2 = 2D, 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed. satellites is
        None when the receiver does not report a count.

        This is the receiver, not the estimate. get_pose_global() reports the
        EKF's fused position and keeps returning one after the GPS degrades.
        """
        return self._gps_fix

    def is_armed(self) -> bool:
        return self._armed

    def recent_statustexts(self, seconds: float = 10.0) -> list:
        """FC STATUSTEXT messages from the last `seconds`: (time, severity, text)."""
        cutoff = time.time() - seconds
        return [entry for entry in self._statustexts if entry[0] >= cutoff]

    # ── Public: blocking utilities ────────────────────────────────────────────

    def _request_streams(self) -> None:
        """Ask the FC to stream position, attitude and GPS status.

        SITL starts with SR0_POSITION=0 (no position stream by default). On an FC
        whose SR* params are already set permanently, this is a harmless no-op, so
        it is always sent.
        """
        ts = self._conn.target_system
        tc = self._conn.target_component

        # Per-message intervals via MAV_CMD_SET_MESSAGE_INTERVAL. REQUEST_DATA_STREAM
        # (below) is deprecated and ArduPilot honours it only as far as the SR*
        # parameters allow — on a link whose SRx_POSITION/SRx_EXTRA1 default to 0 the
        # request is silently dropped and LOCAL_POSITION_NED never arrives. Measured
        # in SITL over TCP 5760: 0 messages in a 4 s window. Everything that reads
        # get_pose_local() then sees a frozen value forever, including any descent
        # gate of the form `alt = -pos[2]`, which will never trigger.
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT):
            self._conn.mav.command_long_send(
                ts, tc, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 100000,   # interval in microseconds → 10 Hz
                0, 0, 0, 0, 0)

        # Belt and braces: harmless on stacks that ignore it, and still the only
        # thing some older ArduPilot builds respond to.
        self._conn.mav.request_data_stream_send(
            ts, tc, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        self._conn.mav.request_data_stream_send(
            ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        self._conn.mav.request_data_stream_send(
            ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10, 1)
        self._log.info(
            'stream rates requested (LOCAL_POSITION_NED + GLOBAL_POSITION_INT '
            '+ ATTITUDE + GPS_RAW_INT @ 10 Hz)')

    def wait_for_connection(self, timeout: float = 30.0) -> None:
        """Block until the FC heartbeat is received."""
        self._log.info('waiting for FCU connection…')
        self._wait_until(lambda: self._connected, timeout, 'FCU connection')
        self._log.info('FCU connected')
        self._request_streams()

    def wait_for_position(self, timeout: float = 15.0) -> tuple[float, float, float]:
        """Block until the first LOCAL_POSITION_NED is received, then return it."""
        self._wait_until(lambda: self._pos_ned is not None, timeout, 'local position')
        return self._pos_ned

    def sleep(self, seconds: float) -> None:
        """Sleep for `seconds`, running the tick loop (re-sends velocity at 20 Hz)."""
        end = time.time() + seconds
        while time.time() < end:
            self._tick()

    def shutdown(self) -> None:
        """Clean up. Call when the mission is done."""
        pass
