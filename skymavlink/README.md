# SkyMAVLink

High-level Python library for controlling ArduPilot drones over MAVLink.

Built and maintained by [SkyRats](https://github.com/SkyRats).

---

## Install

```bash
pip install -r requirements.txt        # runtime dependencies
pip install -e /path/to/skymavlink      # editable install from source
pip install -e '/path/to/skymavlink[test]'   # plus pytest, for tests
```

---

## Quick start

```python
from skymavlink import SkyMAVLink

mav = SkyMAVLink('tcp:127.0.0.1:5760')   # endpoint is required — see Endpoints
mav.wait_for_connection()
mav.set_mode('GUIDED')
mav.arm()
mav.takeoff(1.5)                      # blocks until 1.5 m reached

n, e, d = mav.wait_for_position()
mav.set_local_pose(n + 1.0, e, d)     # fly 1 m North
mav.sleep(5.0)

mav.land()
mav.shutdown()
```

---

## Units and conventions

Every argument below follows these. They are not repeated per method.

| | |
|---|---|
| World frame | **NED** — `north`, `east`, `down` in metres from the EKF origin; `down` is negative above it, so 1.5 m up is `down = -1.5` |
| Body frame | **FRD** — `fwd` (nose), `right` (right wing), `down` (belly) |
| `yaw_deg` | degrees **clockwise from North**: 0 = North, 90 = East |
| `yaw_rate_dps` | degrees/second, clockwise positive |
| Speeds | metres/second |
| `timeout` | seconds; raises `TimeoutError` if the FC does not confirm |
| Returned attitude | **radians** (`get_attitude` only) |

---

## Constructor

```python
SkyMAVLink(endpoint, takeoff_altitude=1.5)
```

| Parameter | Meaning |
|-----------|---------|
| `endpoint` | **required** — pymavlink connection string (see [Endpoints](#endpoints)) |
| `takeoff_altitude` | metres; used by `takeoff()` when called with no argument |

There is no default endpoint. Which link the drone is on — SITL, a router, direct serial
— is a property of the machine you are running on, not of the library, so the caller
states it.

---

## Connection and state

| Signature | Does |
|-----------|------|
| `wait_for_connection(timeout=30.0)` | Block until the FC heartbeat arrives, then request position + attitude streams. Call first. |
| `wait_for_position(timeout=15.0)` | Block until the first position fix; returns `(north, east, down)`. Raise the timeout on a cold EKF — it can take ~40 s. |
| `get_pose_local()` | Last known `(north, east, down)`, or `None` before the first fix. |
| `get_pose_global()` | Last known `(lat, lon, alt_m_agl)` — degrees, degrees, metres above home — or `None`. Stays `None` without a global origin (vision-only, no GPS). |
| `get_attitude()` | Last known `(roll, pitch, yaw)` in radians, or `None`. |
| `get_gps_fix()` | Last known `(fix_type, satellites)`, or `None`. Receiver health — see below. |
| `is_armed()` | `True` if the FC reports armed. |
| `recent_statustexts(seconds=10.0)` | FC messages from the last `seconds` as `(time, severity, text)`. |
| `sleep(seconds)` | Blocking sleep that keeps the link alive and re-sends the active velocity. Use this, never `time.sleep()`. |
| `get_logger()` | The `logging.Logger` the library writes to. |
| `shutdown()` | Cleanup hook. Call when the mission ends. |

State getters return whatever the last update stored, so two calls with no blocking
call between them return the same value.

`get_gps_fix()` reports the **receiver**; `get_pose_global()` reports the EKF's fused
estimate, which keeps arriving after the GPS degrades. Check the fix before trusting a
global position:

```python
fix, sats = mav.get_gps_fix() or (0, None)
if fix < 3:                       # 3 = 3D, 4 = DGPS, 5/6 = RTK
    raise RuntimeError(f'no 3D fix (type {fix}, {sats} sats)')
```

---

## Flight

| Signature | Does |
|-----------|------|
| `set_mode(mode, timeout=15.0)` | Switch mode and wait for confirmation. `mode` is a name like `'GUIDED'`; unknown names raise `ValueError`. |
| `arm(timeout=15.0)` | Arm the motors and wait for confirmation. |
| `takeoff(altitude=None, timeout=30.0)` | Climb to `altitude` metres (default `takeoff_altitude`) and block until 95% of it. |
| `land(speed_ms=None, timeout=60.0)` | Switch to LAND and block until disarmed. `speed_ms` sets descent speed via the `LAND_SPEED` FC param. |
| `rtl()` | Switch to Return-to-Launch. Returns immediately. |
| `kill()` | **Force-disarm now.** In flight the drone falls. Use `rtl()` or `land()` for aborts. |

`mode` accepts: `STABILIZE`, `ACRO`, `ALT_HOLD`, `AUTO`, `GUIDED`, `LOITER`, `RTL`,
`CIRCLE`, `LAND`, `SPORT`, `POSHOLD`, `BRAKE`, `SMART_RTL`.

`takeoff()` is the one blocking call that does **not** raise on timeout — it returns
either way, so check `get_pose_local()` afterwards if it matters.

When `arm()` times out its error carries the FC's recent messages, which name the
pre-arm check that refused:

```
TimeoutError: arm not confirmed within 15s — FC said: PreArm: Need Position Estimate
```

---

## Motion

All return immediately; the drone keeps moving until it arrives or you command
something else.

| Signature | Does |
|-----------|------|
| `set_local_pose(north, east, down, yaw_deg=None)` | Fly to an absolute NED position. `yaw_deg=None` leaves heading alone. |
| `set_body_pose(fwd, right, down, yaw_deg=None)` | Move by an offset relative to the current position and heading. Needs a position fix. |
| `set_body_velocity(fwd, right, down, yaw_rate_dps=0.0)` | Hold a body-frame velocity, re-sent at 20 Hz. `yaw_rate_dps=0` holds the current heading. |
| `set_global_pose(lat, lon, alt_m_agl)` | Fly to a GPS coordinate. `lat`/`lon` decimal degrees, `alt_m_agl` metres above home. Needs a GPS-capable EKF. |

```python
mav.set_local_pose(2.0, 0.0, -1.5)                     # 2 m North at 1.5 m altitude
mav.set_local_pose(0.0, 1.0, -1.5, yaw_deg=90.0)       # 1 m East, facing East
mav.set_body_pose(1.0, 0.0, 0.0)                       # 1 m toward the nose
mav.set_body_pose(0.0, 0.0, -0.5)                      # climb 0.5 m
mav.set_body_velocity(0.5, 0.0, 0.0)                   # 0.5 m/s forward
mav.set_body_velocity(0.3, 0.0, 0.0, yaw_rate_dps=15)  # forward while rotating CW
mav.set_global_pose(38.7169, -9.1399, 10.0)            # GPS, 10 m AGL
```

### Ending a velocity leg

Command zero, then give it time to brake:

```python
mav.set_body_velocity(0.5, 0.0, 0.0)
mav.sleep(3.0)
mav.set_body_velocity(0.0, 0.0, 0.0)   # brake
mav.sleep(2.0)
```

A position command cancels an active velocity, as do `land()` and `kill()`.

To fly an exact distance, close the loop on `get_pose_local()`.

---

## Companion-computer commands

| Signature | Does |
|-----------|------|
| `set_servo(channel, pwm)` | Drive a servo output. `pwm` in microseconds, 800–2200. |
| `send_statustext(text, severity=6)` | Show a message on the ground station. Truncated to 50 bytes. |
| `set_parameter(name, value)` | Set a numeric FC parameter. Fire-and-forget — no ack is awaited. |

```python
mav.set_servo(9, 1900)                     # payload release
mav.send_statustext('target acquired')     # appears in the GCS message log
mav.set_parameter('WPNAV_SPEED', 250.0)    # cm/s
```

`set_servo` needs `SERVOx_FUNCTION = 0` on that output, so the FC hands the channel
over to MAVLink.

---

## Endpoints

| Endpoint | Use |
|----------|-----|
| `udpout:127.0.0.1:14552` | behind a router's UDP fan-out |
| `tcp:127.0.0.1:5760` | SITL, direct |
| `serial:/dev/ttyTHS1:921600` | companion-computer UART, when nothing else needs the link |
| `serial:/dev/serial/by-id/usb-ArduPilot_Pixhawk1-if00:115200` | USB, by stable device id |

Ports and device names are examples — they are whatever your router and companion
computer are configured for.

Over USB, prefer a `/dev/serial/by-id/...` path over `/dev/ttyACM0`: the `ttyACM`
number depends on enumeration order and changes when another device is plugged in.
Run `ls /dev/serial/by-id/` to find it.

A serial port can only be opened by one process. If anything else needs the same link,
put a MAVLink router — [mavp2p](https://github.com/bluenviron/mavp2p), mavlink-router,
MAVProxy — in front of it; it fans one FC link out to several UDP clients:

```
FC serial ─► router ├─► udp:14551 ─► other client
                    └─► udp:14552 ─► SkyMAVLink
```

SkyMAVLink sends a 1 Hz heartbeat so the router learns its return address and routes FC
replies back to it. It transmits as **sysid 1, component 192**
(`MAV_COMP_ID_ONBOARD_COMPUTER2`) — deliberately *not* 191, the default for MAVROS.
Two endpoints sharing one MAVLink address makes targeted FC replies (`COMMAND_ACK`)
ambiguous to the router, so give every client on the link its own component id.

Only one source should command setpoints at a time — two controllers fighting in
GUIDED gives the EKF contradictory targets.

---

## Error handling

```python
mav = SkyMAVLink('tcp:127.0.0.1:5760')
try:
    mav.wait_for_connection()
    mav.set_mode('GUIDED')
    mav.arm()
    mav.takeoff(1.5)
    # ... mission ...
    mav.land()
except TimeoutError as e:
    mav.get_logger().error(f'timeout: {e}')
    mav.rtl()
except KeyboardInterrupt:
    mav.rtl()
finally:
    mav.shutdown()
```

---

## Internal structure

| Module | Holds |
|--------|-------|
| `core.py` | connection, message loop, state, `sleep`, logger |
| `flight.py` | `set_mode`, `arm`, `kill`, `takeoff`, `land`, `rtl` |
| `motion.py` | `set_local_pose`, `set_body_pose`, `set_body_velocity`, `set_global_pose` |
| `commands.py` | `set_servo`, `send_statustext`, `set_parameter` |
| `__init__.py` | `SkyMAVLink(_Flight, _Motion, _Commands)` |

The message loop runs at ~20 Hz inside every blocking call. No background threads.

See `CLAUDE.md` for the reasoning behind these rules (frames, tick loop, transport,
safety) — read it before extending the library.

---

## Requirements

- Python ≥ 3.10
- pymavlink, pyserial
- ArduPilot flight controller in GUIDED mode
