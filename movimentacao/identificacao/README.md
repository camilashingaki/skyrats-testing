# movimentacao/identificacao

Flies the same locked-heading square as `../quadrado*`, but takes a photo
on a fixed cadence and feeds every one of them into the base-detection model
along the way. This script only identifies -- it never centers on or lands
on top of a detected base, it just lands wherever the square happened to end.
For centering + precision landing, see `../../missions/precision_landing/`.

Two backends, same mission:

| Script | Detector |
|---|---|
| `quadrado_identificacao.py` (SkyMAVLink) | `base_detection/raw_detector.py`'s torch-free `RawDetector` -- NCNN by default, or `.tflite` |
| `quadrado_identificacao_nectar.py` (Nectar) | `nectar.ai.detection.Detector` -- also loads the NCNN dir/`.tflite` path, but through `ultralytics.YOLO()`, so it pulls in `ultralytics`/`torch` |

## Setup

**SkyMAVLink:**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install pymavlink pyserial
git clone https://github.com/SkyRats/sky_mavlink.git ../../sky_mavlink   # or wherever you keep it
pip install -e ../../sky_mavlink   # SkyMAVLink itself (not on PyPI)
```

**Nectar** -- `nectar-sdk` is a ROS 2 package, not pip-installable into an
ordinary venv. Set it up with the SDK's own installer first (see
`requirements-nectar.txt` and `../../movimentacao/README.md`'s Setup
section for the exact commands), then `pip install ultralytics` (and,
inside that same environment) run `quadrado_identificacao_nectar.py`.

Model weights already live at `../../base_detection/models/` -- nothing to
export or download for a first run, for either backend.

## Usage

```bash
# SkyMAVLink
python quadrado_identificacao.py
python quadrado_identificacao.py --side 3.0 --classes shape_hexagon shape_star shape_triangle
python quadrado_identificacao.py --model ../../base_detection/models/best_w8a32.tflite

# Nectar
python quadrado_identificacao_nectar.py
python quadrado_identificacao_nectar.py --side 3.0 --classes shape_hexagon shape_star shape_triangle
python quadrado_identificacao_nectar.py --model ../../base_detection/models/best_w8a32.tflite
```

Output goes to `--out-dir` (default `identificacao_output/`):

```
identificacao_output/
  fotos/        every captured frame
  deteccoes/    annotated frames for the ones the model matched
```

Run from inside this folder so the sibling `_util.py` / `nectar_util.py`
(and, through `_util.py`, `base_detection/raw_detector.py`) is importable.
