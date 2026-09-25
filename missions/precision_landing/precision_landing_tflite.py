#!/usr/bin/env python3
"""
Precision landing on the base -- TFLite backend.

Same mission as precision_landing_ncnn.py -- locked-heading search square,
per-frame detection, PID-centering + descent, land on the base or wherever
it ends up if not found -- but loads the TFLite export (best_w8a32.tflite)
through base_detection/raw_detector.py's ai-edge-litert backend instead of
ncnn. Pick whichever backend runs faster/lighter on your target hardware;
both decode the same checkpoint the same way (see
../../base_detection/model_export/README.md).

See common.py in this folder for the full search/centering implementation,
shared with precision_landing_ncnn.py.

Usage:
    python precision_landing_tflite.py
    python precision_landing_tflite.py --connection serial:/dev/ttyACM0:115200 --side 3.0
    python precision_landing_tflite.py --classes shape_hexagon shape_star shape_triangle
"""

from common import DEFAULT_TFLITE_MODEL, run

if __name__ == "__main__":
    run(str(DEFAULT_TFLITE_MODEL), "TFLite")
