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

`SkyRats/sky_mavlink` is a private repository, so this script talks MAVLink
directly through `pymavlink.mavutil` (the library sky_mavlink itself wraps)
instead of importing it. Every vehicle command lives in the `MavlinkDrone`
class; if `sky_mavlink` exposes its own high-level drone class, only that
class's method bodies need to change -- the mission logic in `main()` stays
the same. Camera capture uses plain OpenCV and inference uses
`ultralytics.YOLO` directly.

```bash
pip install pymavlink opencv-python ultralytics

python detect_base_sky_mavlink.py --connection udp:127.0.0.1:14550 --model models/best.pt
python detect_base_sky_mavlink.py --connection /dev/ttyACM0 --model models/best.pt --classes base
```

## Common options (both scripts)

| Flag | Meaning |
|---|---|
| `--model` | Path to the YOLO `best.pt` weights (default `models/best.pt`) |
| `--conf` | Minimum detection confidence (default `0.5`) |
| `--classes` | Class names counted as "base"; omit to accept any class |
| `--confirm-frames` | Consecutive positive frames required before confirming (default `3`) |
| `--height` | Takeoff altitude in meters (default `2.0`) |
| `--speed` | Forward speed in m/s (default `0.3`) |
| `--step-duration` | Seconds flown forward per control step (default `0.5`) |
| `--search-timeout` | Max seconds spent searching before giving up and landing (default `60`) |
| `--snapshot` | Where to save the annotated detection frame (default `base_detected.jpg`) |

Both scripts only fly forward and identify the base (stop, hover, log the
detection, save a snapshot, then land) -- they do not attempt precision
landing on top of the target.
