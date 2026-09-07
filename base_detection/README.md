# Base detection

Two standalone mission scripts: the drone takes off, flies forward, runs a
YOLO model on the live camera feed, and as soon as the base is confirmed for
a few consecutive frames it switches to a centering phase that visually
servos the base into the middle of the frame, descends while centered, and
lands on top of it.

Put your trained weights at `models/best.pt` (or pass `--model <path>`). Both
`Detector` classes load the model through `ultralytics.YOLO`, which
auto-detects the format from the path, so `--model` also accepts the
NCNN/TFLite exports committed under `models/` (`best_ncnn_model/`,
`best_w8a32.tflite`) -- faster on a Raspberry Pi's ARM CPU than plain
PyTorch, especially on a Pi 4. See `model_export/README.md` for how they were
produced, the size/dependency tradeoffs, and how to re-export after
retraining.

`raw_detector.py` is a second, torch-free `Detector` implementation for the
same NCNN/TFLite exports (`ncnn`/`ai-edge-litert` only, no `ultralytics`/
`torch` import at all -- see model_export/README.md for the RAM numbers this
actually saves). It only matches the shape classes (`shape_hexagon`,
`shape_star`, `shape_triangle`), ignoring the number classes on purpose: a
base is one shape with a number printed inside it, so both fire on the same
object, and counting both would read one base as two. It returns a centroid
instead of a box, since that's the only thing `center_and_land()` ever reads
from a detection. Not wired into `detect_base_*.py` yet -- test it first with
`test_raw_detector.py` (see `teste_imagens/README.md`), then swap the
`Detector` import in whichever mission script if it holds up.

Both scripts default to talking to the flight controller over the Raspberry
Pi's **UART** pins (`/dev/serial0` @ 921600 baud), not USB or SITL. Before
running: enable the Pi's serial port hardware and disable its login shell
via `raspi-config` (Interface Options -> Serial Port), and on the FC set the
matching `SERIALx_PROTOCOL=2` (MAVLink2) and `SERIALx_BAUD` for whichever
port is wired to the Pi. `/dev/serial0` is a symlink to the Pi's primary
UART (`ttyAMA0` or `ttyS0` depending on model/config) -- pass
`--connection`/`--baud` (nectar) or `--connection` (sky_mavlink, baud is
embedded in the endpoint string) to point at a different device or, for
bench testing without hardware, back at SITL (`tcp:127.0.0.1:5760` /
`--drone mavlink --connection tcp:127.0.0.1:5762`).

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

python detect_base_nectar.py --model models/best.pt                              # UART, /dev/serial0 @ 921600
python detect_base_nectar.py --model models/best.pt --connection /dev/ttyAMA0 --baud 57600
python detect_base_nectar.py --model models/best.pt --drone mavlink --connection tcp:127.0.0.1:5762  # SITL
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

python detect_base_sky_mavlink.py --model models/best.pt                                     # UART, /dev/serial0 @ 921600
python detect_base_sky_mavlink.py --connection serial:/dev/ttyAMA0:57600 --model models/best.pt
python detect_base_sky_mavlink.py --connection tcp:127.0.0.1:5760 --model models/best.pt      # SITL
```

`--connection` takes any SkyMAVLink/pymavlink endpoint: `serial:/dev/tty...:baud`
(UART/USB), `tcp:host:port` (SITL), or `udpout:host:port` (behind a router).

**Centering**: SkyMAVLink ships no PID utility, so a small PID is rolled in
the script and wired to the exact continuous-correction primitive its README
recommends -- `set_body_velocity(fwd, right, down)` re-sent via `sleep()` --
driving the base's normalized pixel offset from the image center to zero,
descending only while centered, then hands off to `drone.land()` once low
enough.

## Common options (both scripts)

| Flag | Meaning |
|---|---|
| `--connection` | Flight controller link (default `/dev/serial0` UART, sky_mavlink writes it as `serial:/dev/serial0:921600`) |
| `--model` | Path to the YOLO `best.pt` weights (default `models/best.pt`) |
| `--conf` | Minimum detection confidence (default `0.5`) |
| `--classes` | Class names counted as "base"; omit to accept any class |
| `--confirm-frames` | Consecutive positive frames required before confirming (default `3`) |
| `--height` | Takeoff altitude in meters (default `2.0` for nectar, `1.5` for sky_mavlink) |
| `--speed` | Forward speed in m/s (default `0.3`) |
| `--search-timeout` | Max seconds spent searching before giving up and landing (default `60`) |
| `--snapshot` | Where to save the annotated detection frame (default `base_detected.jpg`) |

`detect_base_nectar.py` also takes `--step-duration` (seconds flown forward
per `move_velocity` call) and `--baud` (default `921600`, separate from
`--connection` since Nectar's mavlink/px4_mavlink backends take baud as its
own field); `detect_base_sky_mavlink.py` takes `--step` (seconds serviced per
search-loop iteration via `drone.sleep()`) instead of `--step-duration`,
since SkyMAVLink re-sends the velocity setpoint continuously rather than
per-call, and has no separate `--baud` since SkyMAVLink endpoints embed it
(`serial:/dev/serial0:921600`).

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
