#!/usr/bin/env python3
"""
Precision landing on the base -- NCNN backend.

Flies a locked-heading search square, running the NCNN-exported
base-detection model (torch-free, via base_detection/raw_detector.py) on
every camera frame; once confirmed for --confirm-frames consecutive frames,
switches to a PID centering + descent phase (visual servo on
set_body_velocity) and lands on top of it. Falls back to landing wherever it
currently is if the base is never found or gets lost during the approach.

See common.py in this folder for the full search/centering implementation,
shared with precision_landing_tflite.py -- the only difference between the
two is which model format loads by default.

Usage:
    python precision_landing_ncnn.py
    python precision_landing_ncnn.py --connection serial:/dev/ttyACM0:115200 --side 3.0
    python precision_landing_ncnn.py --classes shape_hexagon shape_star shape_triangle
"""

from common import DEFAULT_NCNN_MODEL, run

if __name__ == "__main__":
    run(str(DEFAULT_NCNN_MODEL), "NCNN")
