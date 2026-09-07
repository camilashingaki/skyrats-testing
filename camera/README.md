# camera

Camera-only mission scripts -- no detection model, just capturing/viewing
what the drone sees. Built on `skymavlink.SkyMAVLink` (see `../skymavlink/`)
and plain OpenCV.

| Script | Does |
|---|---|
| `camera_live_view.py` | Takeoff and hover, showing the live camera feed in a window until `q` is pressed or a timeout is hit, then land. |
| `camera_quadrado_fotos.py` | Takeoff, fly a 2 m square (heading locked, same pattern as `../movimentacao/quadrado.py`), saving a photo every `--photo-period` seconds along the way, land. |

## Setup

```bash
pip install -r requirements.txt
pip install -r ../skymavlink/requirements.txt
pip install -e ../skymavlink
```

## Usage

```bash
python camera_live_view.py                                  # SITL, needs a display
python camera_live_view.py --connection serial:/dev/ttyACM0:115200 --height 1.5

python camera_quadrado_fotos.py --side 2.0 --out-dir fotos
```

`camera_live_view.py` needs a display -- run it on a companion computer with
HDMI/VNC, or over X11 forwarding, and make sure `opencv-python` (not
`opencv-python-headless`) is installed. `camera_quadrado_fotos.py` has no
display dependency; it only writes files to `--out-dir`.

Run scripts from inside this folder so the sibling `_util.py` is importable.
