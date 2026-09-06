# Base detection

Two standalone mission scripts: the drone takes off, flies forward, runs a
YOLO model on the live camera feed, and stops/hovers as soon as the base is
confirmed for a few consecutive frames.

Put your trained weights at `models/best.pt` (or pass `--model <path>`).

## `detect_base_nectar.py` -- Black-Bee-Drones/nectar-sdk

Uses `nectar.control.DroneFactory` for flight (arm/takeoff/velocity/land) and
`nectar.ai.detection.Detector` for YOLO inference (auto-detects the
Ultralytics framework from `best.pt`). Works with any Nectar backend
(`mavlink`, `mavros`, `px4`, `px4_mavlink`, `px4_dds`).

```bash
pip install nectar-sdk opencv-python ultralytics

python detect_base_nectar.py --model models/best.pt --drone mavlink
python detect_base_nectar.py --model models/best.pt --drone mavros --env indoor
python detect_base_nectar.py --model models/best.pt --classes base --conf 0.6
```

## `detect_base_sky_mavlink.py` -- SkyRats/sky_mavlink

Uses the real `SkyMAVLink` class (`arm`, `takeoff`, `set_body_velocity`,
`land`, ...). SkyMAVLink runs no background thread -- its message loop only
advances inside blocking calls -- so the search loop calls `drone.sleep(step)`
on every iteration, which both services the link and re-sends the active
`set_body_velocity` forward setpoint. Camera capture uses plain OpenCV and
inference uses `ultralytics.YOLO` directly (SkyMAVLink only handles the
MAVLink side, not vision).

```bash
pip install -r requirements.txt
pip install -e /path/to/sky_mavlink   # SkyMAVLink itself (not on PyPI)

python detect_base_sky_mavlink.py --connection tcp:127.0.0.1:5760 --model models/best.pt
python detect_base_sky_mavlink.py --connection serial:/dev/ttyACM0:115200 --model models/best.pt --classes base
```

`--connection` takes any SkyMAVLink/pymavlink endpoint: `tcp:host:port` (SITL),
`udpout:host:port` (behind a router), or `serial:/dev/tty...:baud`.

## Common options (both scripts)

| Flag | Meaning |
|---|---|
| `--model` | Path to the YOLO `best.pt` weights (default `models/best.pt`) |
| `--conf` | Minimum detection confidence (default `0.5`) |
| `--classes` | Class names counted as "base"; omit to accept any class |
| `--confirm-frames` | Consecutive positive frames required before confirming (default `3`) |
| `--height` | Takeoff altitude in meters (default `2.0` for nectar, `1.5` for sky_mavlink) |
| `--speed` | Forward speed in m/s (default `0.3`) |
| `--search-timeout` | Max seconds spent searching before giving up and landing (default `60`) |
| `--snapshot` | Where to save the annotated detection frame (default `base_detected.jpg`) |

`detect_base_nectar.py` also takes `--step-duration` (seconds flown forward
per `move_velocity` call); `detect_base_sky_mavlink.py` takes `--step`
(seconds serviced per search-loop iteration via `drone.sleep()`) instead,
since SkyMAVLink re-sends the velocity setpoint continuously rather than
per-call.

Both scripts only fly forward and identify the base (stop, hover, log the
detection, save a snapshot, then land) -- they do not attempt precision
landing on top of the target.
