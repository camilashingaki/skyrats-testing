# movimentacao

Basic flight scripts, in two backends: `skymavlink.SkyMAVLink` (see
`../skymavlink/`) and the Nectar SDK (`Black-Bee-Drones/nectar-sdk`). Each
one is a standalone smoke test -- run them in order when bringing up a new
vehicle/link.

| Mission | SkyMAVLink | Nectar |
|---|---|---|
| Takeoff to ~3 m, print altitude every second while hovering, land. No lateral movement. | `takeoff_pouso.py` | `takeoff_pouso_nectar.py` |
| Takeoff, fly 5 m forward, land. | `andar_frente.py` | `andar_frente_nectar.py` |
| Takeoff, fly a 2 m square, land. Heading locked the whole time (no yawing between legs). | `quadrado.py` | `quadrado_nectar.py` |

Both backends do the same three missions; pick whichever stack your vehicle
already runs. They are not interchangeable at runtime -- SkyMAVLink talks
pymavlink directly, Nectar wraps ROS 2 -- so don't mix `--connection` flags
between them.

## Setup

**SkyMAVLink** -- a plain Python venv:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r ../skymavlink/requirements.txt
pip install -e ../skymavlink
```

**Nectar** -- `nectar-sdk` is a ROS 2 package (Humble/Jazzy/Kilted), not
pip-installable into an ordinary venv (it depends on `rclpy`). Set it up
with the SDK's own installer first, then run the `*_nectar.py` scripts
inside that same environment:

```bash
# fresh machine, no ROS 2 yet
bash <(curl -fsSL https://raw.githubusercontent.com/Black-Bee-Drones/nectar-sdk/main/scripts/bootstrap.sh)

# already have ROS 2
cd ~/ros2_ws/src && git clone https://github.com/Black-Bee-Drones/nectar-sdk.git
cd nectar-sdk && make setup
```

See `requirements-nectar.txt` and the [Nectar installation guide](https://black-bee-drones.github.io/nectar-sdk/setup/) (a Docker path that needs no host ROS 2 install is documented there too).

## Usage

All scripts default to SITL so an accidental run never moves a real drone
(SkyMAVLink: `tcp:127.0.0.1:5760`; Nectar's `mavlink` backend: `tcp:127.0.0.1:5762`).
Point `--connection` at real hardware explicitly:

```bash
# SkyMAVLink
python takeoff_pouso.py
python andar_frente.py --distance 5.0
python quadrado.py --side 2.0
python quadrado.py --connection serial:/dev/ttyACM0:115200 --side 2.0     # real hardware

# Nectar
python takeoff_pouso_nectar.py
python andar_frente_nectar.py --distance 5.0
python quadrado_nectar.py --side 2.0
python quadrado_nectar.py --connection /dev/serial0 --baud 921600 --side 2.0   # real hardware
```

Run them from inside this folder (or `python -m movimentacao.quadrado` from
the repo root) -- each backend imports its own small helper next to it
(`_util.py` for SkyMAVLink, `nectar_util.py` for Nectar).

## Why position commands, not velocity + timing

Both backends fly `andar_frente*`/`quadrado*` with a position/navigation
call (`set_body_pose()` for SkyMAVLink, `move_to()` for Nectar) instead of a
velocity command timed with `sleep()`/`delay()`. Two reasons:

- Distance from `speed * time` drifts with wind/battery sag; distance from
  the FC's own position estimate doesn't (`get_pose_local()` /
  `move_to()`'s internal position polling).
- An explicit, always-sent yaw command holds the heading. Without it,
  ArduPilot's default `WP_YAW_BEHAVIOR` turns the nose toward the direction
  of travel -- exactly the "ficar mexendo o pescoço" (yawing at every
  corner) `quadrado*` is written to avoid. SkyMAVLink does this by passing
  `yaw_deg` explicitly on every `set_body_pose()` call; Nectar does it by
  always sending an explicit (possibly zero) yaw-rate under the hood even
  when `move_to(yaw=None)` is asked to just hold the current heading (see
  `nectar/control/mavlink/transport.py`'s `_VELOCITY_MASK`, which never
  flags yaw-rate "ignore" the way it flags yaw) -- same underlying MAVLink
  mechanism, same fix, in both SDKs.
