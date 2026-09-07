# movimentacao

Basic flight scripts built on `skymavlink.SkyMAVLink` (see `../skymavlink/`).
Each one is a standalone smoke test -- run them in order when bringing up a
new vehicle/link.

| Script | Does |
|---|---|
| `takeoff_pouso.py` | Takeoff to ~3 m, print the altitude every second while hovering, land. No lateral movement. |
| `andar_frente.py` | Takeoff, fly 5 m forward, land. |
| `quadrado.py` | Takeoff, fly a 2 m square, land. Heading is locked at takeoff for the whole square (no yawing between legs). |

## Setup

```bash
pip install -r ../skymavlink/requirements.txt
pip install -e ../skymavlink
```

## Usage

All scripts default to SITL (`tcp:127.0.0.1:5760`) so an accidental run never
moves a real drone. Point `--connection` at real hardware explicitly:

```bash
python takeoff_pouso.py
python andar_frente.py --distance 5.0
python quadrado.py --side 2.0
python quadrado.py --connection serial:/dev/ttyACM0:115200 --side 2.0   # real hardware
```

Run them from inside this folder (or `python -m movimentacao.quadrado` from
the repo root) -- they import the small `_util.py` helper next to them for
distance-based arrival checks (`get_pose_local()`, never `speed * time`, per
`../skymavlink/CLAUDE.md`).

## Why position commands, not velocity + timing

`andar_frente.py` and `quadrado.py` both use `set_body_pose()` (an absolute
position target, explicit `yaw_deg` on every call) instead of
`set_body_velocity()` timed with `sleep()`. Two reasons:

- Distance from `speed * time` drifts with wind/battery sag; distance from
  `get_pose_local()` doesn't.
- An explicit `yaw_deg` on every leg holds the heading. Without it, ArduPilot's
  default `WP_YAW_BEHAVIOR` turns the nose toward the direction of travel --
  exactly the "ficar mexendo o pescoço" (yawing at every corner) `quadrado.py`
  is written to avoid.
