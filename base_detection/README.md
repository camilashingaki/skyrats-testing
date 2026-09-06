# Base detection

Two standalone mission scripts: the drone takes off, flies forward, runs a
YOLO model on the live camera feed, and as soon as the base is confirmed for
a few consecutive frames it switches to a centering phase that visually
servos the base into the middle of the frame, descends while centered, and
lands on top of it.

Put your trained weights at `models/best.pt` (or pass `--model <path>`).

Camera mounting assumption for the centering phase (both scripts): a
forward/nadir camera where image columns map to the drone's right and image
rows map to the drone's forward direction. If your camera is mounted
differently, flip `--lateral-sign` and/or `--forward-sign` (`1` or `-1`)
rather than editing the code.

## `detect_base_nectar.py` -- Black-Bee-Drones/nectar-sdk

Uses `nectar.control.DroneFactory` for flight (arm/takeoff/velocity/land) and
`nectar.ai.detection.Detector` for YOLO inference (auto-detects the
Ultralytics framework from `best.pt`). Works with any Nectar backend
(`mavlink`, `mavros`, `px4`, `px4_mavlink`, `px4_dds`).

**Centering**: two `nectar.control.pid.PIDController` instances -- Nectar's
own documented "usable standalone for any control loop" primitive -- drive
the base's normalized pixel offset from the image center to zero via
`move_velocity(reference=BODY)`, descending only while centered, then hands
off to `drone.land()` once low enough.

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

**Centering**: SkyMAVLink ships no PID utility, so a small PID is rolled in
the script and wired to the exact continuous-correction primitive its README
recommends -- `set_body_velocity(fwd, right, down)` re-sent via `sleep()` --
driving the base's normalized pixel offset from the image center to zero,
descending only while centered, then hands off to `drone.land()` once low
enough.

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

### Centering / landing options (both scripts)

| Flag | Meaning |
|---|---|
| `--align-tolerance` | Normalized pixel error (0-1) below which the base counts as centered (default `0.15`) |
| `--land-altitude` | Altitude (m) at which centering hands off to `land()` (default `0.4`) |
| `--descent-speed` | Descent speed (m/s) while centered (default `0.15`) |
| `--landing-timeout` | Max seconds spent centering/descending before landing anyway (default `45`) |
| `--max-lost-frames` | Frames without a matching detection tolerated during descent before giving up and landing (default `15`) |
| `--kp` / `--ki` / `--kd` | PID gains shared by both centering axes (default `0.6` / `0.0` / `0.05`) |
| `--max-correction` | Max horizontal correction speed in m/s (default `0.4`) |
| `--lateral-sign` / `--forward-sign` | `1` or `-1`; flip if the drone drifts the wrong way while centering |

If the base is lost for more than `--max-lost-frames` during descent, or the
landing phase exceeds `--landing-timeout`, the script gives up centering and
lands wherever it currently is rather than continuing to drift blindly.
