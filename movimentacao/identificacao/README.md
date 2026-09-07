# movimentacao/identificacao

Flies the same locked-heading square as `../quadrado.py`, but takes a photo
on a fixed cadence and feeds every one of them into the base-detection model
along the way. This script only identifies -- it never centers on or lands
on top of a detected base, it just lands wherever the square happened to end.
For centering + precision landing, see `../../missions/precision_landing/`.

Reuses `base_detection/raw_detector.py`'s torch-free `RawDetector` (see
`../../base_detection/README.md`) instead of duplicating YOLO inference:
NCNN by default (`models/best_ncnn_model/`), or pass `--model
.../best_w8a32.tflite` for the TFLite export.

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
python quadrado_identificacao.py
python quadrado_identificacao.py --side 3.0 --classes shape_hexagon shape_star shape_triangle
python quadrado_identificacao.py --model ../../base_detection/models/best_w8a32.tflite   # TFLite instead of NCNN
```

Output goes to `--out-dir` (default `identificacao_output/`):

```
identificacao_output/
  fotos/        every captured frame
  deteccoes/    annotated frames for the ones the model matched
```

Run from inside this folder so the sibling `_util.py` (and, through it,
`base_detection/raw_detector.py`) is importable.
