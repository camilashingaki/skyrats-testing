# missions/precision_landing

The final mission: takeoff, fly a locked-heading search square, and as soon
as the base is confirmed for a few consecutive frames, visually servo onto
it (PID centering on `set_body_velocity`) and land on top of it. Falls back
to landing wherever it currently is if the base is never found, or gets lost
during the final approach.

Two entry points, same mission, different model backend:

| Script | Model |
|---|---|
| `precision_landing_ncnn.py` | `base_detection/models/best_ncnn_model/` (NCNN) |
| `precision_landing_tflite.py` | `base_detection/models/best_w8a32.tflite` (TFLite) |

Both are thin wrappers around `common.py`, which holds the actual
search/centering/landing logic -- see its module docstring for how it
relates to `base_detection/detect_base_sky_mavlink.py` (where the PID
centering pattern comes from) and `base_detection/raw_detector.py` (the
torch-free inference both scripts run per frame).

## How it differs from `base_detection/detect_base_sky_mavlink.py`

That script flies a single straight line forward while searching.
`common.py` here flies a **locked-heading square** instead (reusing the
`movimentacao/quadrado.py` pattern) so the vehicle actually covers an area
rather than a line, and swaps the ultralytics-based `Detector` for
`base_detection/raw_detector.py`'s torch-free `RawDetector` (NCNN or
TFLite) -- no `ultralytics`/`torch` dependency needed on the flight
computer. The PID-centering/landing phase itself is carried over close to
unchanged.

## Setup

```bash
pip install -r requirements.txt
pip install -r ../../skymavlink/requirements.txt
pip install -e ../../skymavlink
```

Model weights already live at `../../base_detection/models/` -- nothing to
export or download for a first run.

## Usage

```bash
python precision_landing_ncnn.py                                              # SITL, tcp:127.0.0.1:5760
python precision_landing_ncnn.py --connection serial:/dev/ttyACM0:115200      # real hardware
python precision_landing_tflite.py --side 4.0 --speed 0.25

python precision_landing_ncnn.py --classes shape_hexagon                      # only land on hexagon bases
```

## Options

| Flag | Meaning |
|---|---|
| `--connection` | SkyMAVLink endpoint (default SITL) |
| `--model` | Path to the model (default: this script's backend, at its default location) |
| `--conf` | Minimum detection confidence (default `0.5`) |
| `--classes` | Class names counted as "base"; default accepts any shape class `RawDetector` matches |
| `--confirm-frames` | Consecutive positive frames required before confirming (default `3`) |
| `--height` | Search altitude in meters (default `1.5`) |
| `--side` | Search-square side length in meters (default `3.0`) |
| `--speed` | Search speed in m/s on each leg (default `0.3`) |
| `--search-timeout` | Max seconds spent searching before giving up and landing (default `90`) |
| `--snapshot` | Where to save the annotated detection frame (default `base_detected.jpg`) |

### Centering / landing options

| Flag | Meaning |
|---|---|
| `--align-tolerance` | Normalized pixel error (0-1) below which the base counts as centered (default `0.15`) |
| `--land-altitude` | Altitude (m) at which centering hands off to `land()` (default `0.4`) |
| `--descent-speed` | Descent speed (m/s) while centered (default `0.15`) |
| `--landing-timeout` | Max seconds spent centering/descending before landing anyway (default `45`) |
| `--max-lost-frames` | Frames without a matching detection tolerated during descent (default `15`) |
| `--kp` / `--ki` / `--kd` | PID gains shared by both centering axes (default `0.6` / `0.0` / `0.05`) |
| `--max-correction` | Max horizontal correction speed in m/s (default `0.4`) |
| `--lateral-sign` / `--forward-sign` | `1` or `-1`; flip if the drone drifts the wrong way while centering |

Camera mounting assumption for the centering phase: a forward/nadir camera
where image columns map to the drone's right and image rows map to the
drone's forward direction. If your camera is mounted differently, flip
`--lateral-sign`/`--forward-sign`.
