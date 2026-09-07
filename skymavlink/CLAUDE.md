# SkyMAVLink — Library Rules

## What this is

`skymavlink.SkyMAVLink` is the single class missions import. It is a **pure Python
pymavlink library** — not a ROS node, not a colcon package.

```python
class SkyMAVLink(_Flight, _Motion)   # __init__.py
    _Flight(_Core)                   # flight.py   — set_mode, arm, kill, takeoff, land, rtl
    _Motion(_Core)                   # motion.py   — set_local_pose, set_body_pose,
                                     #               set_body_velocity, set_global_pose
    _Commands(_Core)                 # commands.py — set_servo, send_statustext, set_parameter
    _Core                            # core.py     — connection, _drain, _tick, state, sleep
```

All three mixins inherit `_Core`; the diamond collapses through MRO onto one
`_Core.__init__`. Put new commands in a mixin, never in `__init__.py`.

**No rclpy. No MAVROS. No threads.** An earlier design was a ROS node driving MAVROS
topics; it was removed. Anything in this repo that still mentions `rclpy.Node`,
`SingleThreadedExecutor`, `/mavros/...` topics, ENU, or FLU is a leftover and must be
rewritten, not extended.

## Frames: NED and FRD — the inverse of the rest of sky_ws2

```
World NED:  +N North, +E East, +D Down    → altitude = −down  (1.5 m up ⇒ down = −1.5)
Body FRD:   +F nose,  +R right, +D belly
Yaw:        radians CW from North          (0 = North, π/2 = East)
yaw_deg args: degrees CW from North        (0 = North, 90 = East)
```

This is the opposite convention from `sky_vision2`, whose bridge publishes **ENU/FLU**
to `/mavros/vision_pose/pose` (see `sky_ws2/.claude/rules/coordinate_frames.md`).
MAVROS converts ENU→NED on the way in; SkyMAVLink talks NED to the FC directly and
converts nothing. Never copy an ENU snippet from `sky_vision2` into this repo.

`set_body_pose` is not a body-frame message — it reads `_pos_ned` + `_yaw_ned`,
rotates the FRD offset into NED itself, and sends an **absolute** `MAV_FRAME_LOCAL_NED`
target. It requires a position fix and returns with a warning if `_pos_ned is None`.
`set_body_velocity` is the real thing: `MAV_FRAME_BODY_NED`, rotated by the FC.

## The tick loop is the whole concurrency model

`_tick()` does one ~50 ms cycle: drain incoming messages → send heartbeat if 1 s
elapsed → re-send the active velocity setpoint. It runs **only while mission code is
inside a blocking library call** (`sleep`, `wait_for_*`, `_wait_until`, `_retry_until`,
`takeoff`, `land`, `arm`, `set_mode`).

Consequences that mission code must respect:

- **Never `time.sleep()` in a mission — always `mav.sleep()`.** Under `time.sleep()`
  no ticks run, the velocity setpoint stops going out, ArduPilot's `GUID_TIMEOUT`
  (default 3 s) fires, and the drone brakes to a hold.
- State getters (`get_pose_local`, `get_pose_global`, `get_attitude`, `is_armed`) return whatever the last
  tick stored. Back-to-back calls with no blocking call between them return identical
  stale values.
- The 1 Hz heartbeat exists so **mavp2p** learns our UDP return address and routes FC
  replies back. Long non-blocking stretches starve it.

## Transport

```
FC serial ─► router ├─► udp:14551 ─► MAVROS      (vision bridge → EKF3)
                    └─► udp:14552 ─► SkyMAVLink  (mode, arm, setpoints, state)
```

`endpoint` is a **required** constructor argument — there is no library default, and
do not reintroduce one. The transport is a property of the machine, not of the library;
callers pass `udpout:127.0.0.1:14552` (behind a router), `tcp:127.0.0.1:5760` (SITL
direct) or `serial:/dev/ttyTHS1:921600` (companion UART straight to the FC, only when
nothing else needs the link). The serial port is exclusive — anything else needing the
same link has to go through a MAVLink router (mavp2p, mavlink-router, MAVProxy). Test
scripts set `ENDPOINT` as a constant at the top, always SITL, so an accidental run
cannot move a real drone.

`serial:` is ours, not pymavlink's. `_parse_endpoint()` splits it into a device path
and a `baud` keyword before `mavlink_connection()` sees it; passed through whole,
pymavlink reads the colons as a UDP host:port and raises. Every other form is passed
through untouched. Keep the split in `_parse_endpoint` — do not scatter serial parsing
into the constructor.

Over USB, point `serial:` at `/dev/serial/by-id/...`, not `/dev/ttyACM0` — the ACM
number depends on enumeration order.

`source_system=1, source_component=192` (`MAV_COMP_ID_ONBOARD_COMPUTER2`); target is
system 1 / component 1. **Do not use 191** — that is what MAVROS transmits as, and two
endpoints sharing one MAVLink address make targeted FC replies (`COMMAND_ACK`)
ambiguous to the router. `_drain()` drops any message whose `srcSystem` is not the
target, and only trusts `HEARTBEAT` from `srcComponent == 1`, so MAVROS's own
heartbeats on the shared link are ignored.

**MAVROS provides vision pose only. SkyMAVLink provides commands only.** Never publish
setpoints from both — two GUIDED sources fight and the EKF sees contradictory targets.

## Consumed messages

Only six are parsed. Anything else is dropped by `_drain()`.

| Message | Fills |
|---------|-------|
| `HEARTBEAT` | `_connected`, `_armed`, `_mode` |
| `LOCAL_POSITION_NED` | `_pos_ned` (n, e, d) |
| `GLOBAL_POSITION_INT` | `_pos_global` (lat°, lon°, alt m above home) |
| `ATTITUDE` | `_yaw_ned`, `_att_ned` |
| `GPS_RAW_INT` | `_gps_fix` (fix_type, satellites) |
| `STATUSTEXT` | `_statustexts` (50-deep ring of `(time, severity, text)`) |

`_pos_global` keeps `relative_alt` (mm → m), not AMSL `alt`, so `get_pose_global()`
round-trips with `set_global_pose()`, which commands in
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`. It stays `None` with no global origin — a
vision-only EKF never emits `GLOBAL_POSITION_INT`.

`_gps_fix` and `_pos_global` are not interchangeable. `GLOBAL_POSITION_INT` is the
EKF's fused output and keeps arriving after the receiver degrades, so
`get_pose_global()` returning a position is not evidence of a GPS fix — only
`get_gps_fix()` is. `satellites_visible == 255` means "unknown" and is stored as
`None`, not 255.

`wait_for_connection()` calls `_request_streams()` after the first heartbeat. It asks
per message id via `MAV_CMD_SET_MESSAGE_INTERVAL` at 10 Hz, then repeats the request as
the deprecated `MAV_DATA_STREAM_POSITION` (6) and `EXTRA1` (1) for stacks that only
answer the old form. Both are needed: ArduPilot honours `REQUEST_DATA_STREAM` only as
far as the `SR*` parameters allow, and a link whose `SRx_POSITION` defaults to 0 — SITL,
for one — silently drops it, leaving `get_pose_local()` `None` forever. Harmless where
`SR*` is already set permanently on the FC.

## Blocking and timeout semantics

`_wait_until` polls a predicate; `_retry_until` re-sends a command every `period`
seconds until the predicate holds. Both raise `TimeoutError`.

| Method | Confirms against | On failure |
|--------|------------------|------------|
| `wait_for_connection` | `_connected` | `TimeoutError` |
| `wait_for_position` | `_pos_ned is not None` | `TimeoutError` |
| `set_mode` | `_mode == _ACM_MODE[name]` | `TimeoutError` |
| `arm` | `_armed` | `TimeoutError`, with recent `STATUSTEXT` appended |
| `land` | `not _armed` | `TimeoutError` |
| `takeoff` | `_pos_ned[2] <= −0.95 × alt` | **returns anyway, logs reached altitude** |

`takeoff` is the exception — it is fire-and-poll, sent once, and never raises. Missions
that care must assert on `get_pose_local()` afterwards. Keep it that way or change it
deliberately; do not "fix" it silently.

`set_mode` only accepts names in `_ACM_MODE` (ArduCopter custom-mode numbers) and
raises `ValueError` otherwise.

`arm()` catches its own `TimeoutError` and re-raises it with the last four
`STATUSTEXT` lines — that is where ArduPilot names the pre-arm check that refused.
Nothing else reads `_statustexts`.

## type_mask constants (`motion.py`)

Bit set = **ignore** that field. These are the standard MAVLink combinations; do not
hand-edit them.

| Constant | Value | Uses |
|----------|-------|------|
| `_MASK_POS_ONLY` | `0x0FF8` | position |
| `_MASK_POS_YAW` | `0x09F8` | position + yaw |
| `_MASK_VEL` | `0x0DC7` | velocity — **defined but deliberately unused**, see below |
| `_MASK_VEL_YAW_RT` | `0x05C7` | velocity + yaw rate |

## Never flag yaw_rate "ignore" on a velocity setpoint

`set_body_velocity()` always sends `_MASK_VEL_YAW_RT`, even when `yaw_rate_dps` is 0.
Do not "optimise" it back to `_MASK_VEL` when the rate is zero.

Marking yaw_rate as ignored hands yaw to the FC. ArduPilot's
`ModeGuided::set_yaw_state_rad()` sees `use_yaw=false, use_yaw_rate=false`, calls
`auto_yaw.set_mode_to_default()`, and with the default `WP_YAW_BEHAVIOR=2` lands on
`LOOK_AT_NEXT_WP` → `_yaw_angle_rad = pos_control->get_yaw_rad()`. The nose turns into
the direction of travel, so body forward/right rotate *while you are flying a body-frame
pattern* and the frame you commanded in stops existing.

Measured on a 1 m square at 0.8 m/s: **80.7° of heading drift**, wrecking the last two
legs. The same square at 0.3 and 0.5 m/s drifted <0.03° — the effect is speed-dependent,
so it hides in slow testing and appears when you speed up.

Sending an explicit zero rate takes the `use_yaw_rate` branch instead and holds heading
(0.046° drift under otherwise identical conditions). `WP_YAW_BEHAVIOR=0` on the FC fixes
it too (0.038°), but relying on that makes correctness depend on FC configuration the
library does not own.

Any position command clears `_vel_cmd` first, so a pose call implicitly cancels an
active velocity; so do `kill()` and `land()`.

## Ending a velocity leg

Command zero and let the tick loop send it. There is no `stop()` — do not add one:
clearing `_vel_cmd` stops the sending, not the drone, which keeps flying the last
velocity until `GUID_TIMEOUT` (default 3 s) expires.

```python
mav.set_body_velocity(0.0, 0.0, 0.0)   # re-sent at 20 Hz
mav.sleep(2.0)
```

For rotation, lock the heading instead: `mav.set_local_pose(n, e, d, yaw_deg=target)`.

Distances come from `get_pose_local()`, never from `speed × time`.

## Safety

- `kill()` force-disarms with `param2 = 21196` (ArduPilot magic). In flight the drone
  falls. It is not a soft abort — never use it as an error path where `rtl()` fits.
- `land(speed_ms=...)` writes the FC param `LAND_SPEED` in **cm/s** (arg is m/s) and
  sleeps 0.5 s before the mode switch so the FC applies it. The param persists on the
  FC after the flight.
- Commands are sent regardless of arm/GUIDED state; the FC ignores setpoints unless
  GUIDED + armed. Sequencing is the mission's job, not the library's.

## Testing against SITL

```bash
arducopter -w --model quad --speedup 1 \
  --defaults Tools/autotest/default_params/copter.parm \
  --home -35.363262,149.165237,584,353 -I0
```

The binary and the defaults file are both inside an ArduPilot checkout
(`build/sitl/bin/arducopter`, `Tools/autotest/…`); paths depend on where it is cloned.

SITL blocks at "Waiting for connection" until a TCP client attaches, then needs
~30–60 s for EKF/GPS before pre-arm passes — pass a generous `arm(timeout=...)`.
Connect with `SkyMAVLink(endpoint='tcp:127.0.0.1:5760')`.

## See also

- `README.md` — public API reference
- In the companion ROS workspace (`sky_ws2`), when working alongside it: `sky_vision2`'s
  bridge-node rules — the ENU vision-pose half of the stack — and that workspace's
  coordinate-frame rules, on why that half is ENU and this one is NED.
