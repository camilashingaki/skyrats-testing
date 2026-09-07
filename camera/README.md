# camera

Camera-only mission scripts -- no detection model, just capturing/viewing
what the drone sees. Two backends: `skymavlink.SkyMAVLink` (+ plain OpenCV
`VideoCapture`) and the Nectar SDK (+ `nectar.vision.camera.ImageHandler`).

| Mission | SkyMAVLink | Nectar |
|---|---|---|
| Takeoff and hover, showing the live camera feed in a window until `q` is pressed or a timeout is hit, then land. | `camera_live_view.py` | `camera_live_view_nectar.py` |
| Takeoff, fly a 2 m square (heading locked), saving a photo every `--photo-period` seconds along the way, land. | `camera_quadrado_fotos.py` | `camera_quadrado_fotos_nectar.py` |

## Setup

**SkyMAVLink:**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r ../skymavlink/requirements.txt
pip install -e ../skymavlink
```

**Nectar** -- `nectar-sdk` is a ROS 2 package, not pip-installable into an
ordinary venv. Set it up with the SDK's own installer first (see
`requirements-nectar.txt` and `../movimentacao/README.md`'s Setup section
for the exact commands), then run the `*_nectar.py` scripts in that same
environment.

## Usage

```bash
# SkyMAVLink -- SITL by default, needs a display
python camera_live_view.py
python camera_live_view.py --connection serial:/dev/ttyACM0:115200 --height 1.5
python camera_quadrado_fotos.py --side 2.0 --out-dir fotos

# Nectar -- SITL by default, needs a display
python camera_live_view_nectar.py
python camera_live_view_nectar.py --connection /dev/serial0 --baud 921600 --camera-type imx219
python camera_quadrado_fotos_nectar.py --side 2.0 --out-dir fotos
```

`camera_live_view*.py` needs a display -- run on a companion computer with
HDMI/VNC, or over X11 forwarding, with `opencv-python` (not
`opencv-python-headless`) installed. `camera_quadrado_fotos*.py` has no
display dependency; it only writes files to `--out-dir`.

The Nectar scripts' `--camera-type` selects the capture source
(`webcam` / `imx219` / `ros` / a ROS topic / a video file path) -- see
`nectar_util.py`'s `_CAMERA_PARAMS` and
`base_detection/detect_base_nectar.py`'s docstring for details.

Run scripts from inside this folder so the sibling `_util.py` /
`nectar_util.py` is importable.
