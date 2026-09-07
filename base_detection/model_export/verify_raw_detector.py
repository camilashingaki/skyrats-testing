#!/usr/bin/env python3
"""Check raw_detector.py against the trusted ultralytics.YOLO pipeline.

raw_detector.py hand-decodes the NCNN/TFLite output instead of going through
ultralytics.YOLO. This script is the reason to trust that decode: it runs
both on the same synthetic images (a shape drawn over noise, at various
image sizes/positions) and asserts the picked centroid and confidence match
almost exactly. Two things it caught during development, now baked into
raw_detector.py's comments:

- NCNN's exported graph reports boxes in pixel space (relative to the padded
  640x640 input); TFLite/LiteRT normalizes them to [0, 1] instead. Same
  export script, same source checkpoint, different convention per backend.
- This checkpoint's metadata reports end2end=false, which makes
  ultralytics' own decode_bboxes() emit (center_x, center_y, w, h), not
  (x1, y1, x2, y2).

Re-run this after retraining/re-exporting to confirm the decode still holds
(e.g. if a future export sets end2end=true, the box format flips back to
xyxy and raw_detector.py's unpacking would need updating to match).

Requires requirements-export.txt (needs ultralytics) -- dev machine only.
"""

import sys
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from raw_detector import CLASS_NAMES, SHAPE_CLASS_INDICES, RawDetector  # noqa: E402

MODELS_DIR = Path(__file__).resolve().parent.parent / "models"
CENTER_TOLERANCE_PX = 2.0

TEST_CASES = [
    # (width, height, seed, shape_center_x, shape_center_y, shape_radius)
    (800, 600, 1, 420, 300, 90),
    (640, 480, 2, 150, 350, 70),
    (480, 640, 3, 200, 500, 60),
    (1280, 720, 4, 900, 550, 110),
]


def make_test_image(w: int, h: int, seed: int, cx: int, cy: int, r: int) -> np.ndarray:
    """A hexagon-with-a-number over noise, standing in for a photographed base."""
    rng = np.random.default_rng(seed)
    img = rng.integers(40, 90, (h, w, 3)).astype(np.uint8)
    pts = np.array(
        [[cx + r * np.cos(a), cy + r * np.sin(a)] for a in np.linspace(0, 2 * np.pi, 7)[:-1]], dtype=np.int32
    )
    cv2.fillPoly(img, [pts], (200, 200, 30))
    cv2.putText(img, "3", (cx - 25, cy + 25), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (10, 10, 10), 6)
    return img


def reference_best(model: YOLO, img: np.ndarray):
    """Highest-confidence shape_* box from the trusted ultralytics pipeline, at conf=0 (no filtering)."""
    result = model.predict(img, conf=0.0, verbose=False)[0]
    confs = result.boxes.conf.tolist()
    classes = result.boxes.cls.tolist()
    candidates = [i for i in range(len(confs)) if int(classes[i]) in SHAPE_CLASS_INDICES]
    best_i = max(candidates, key=lambda i: confs[i])
    x1, y1, x2, y2 = result.boxes.xyxy[best_i].tolist()
    return CLASS_NAMES[int(classes[best_i])], confs[best_i], ((x1 + x2) / 2, (y1 + y2) / 2)


def main() -> None:
    backends = [MODELS_DIR / "best_ncnn_model", MODELS_DIR / "best_w8a32.tflite"]
    all_ok = True

    for w, h, seed, cx, cy, r in TEST_CASES:
        img = make_test_image(w, h, seed, cx, cy, r)
        print(f"image {w}x{h}, shape drawn at ({cx},{cy}):")

        for backend_path in backends:
            ref_model = YOLO(str(backend_path), task="detect")
            ref_class, ref_conf, (ref_cx, ref_cy) = reference_best(ref_model, img)

            raw = RawDetector(str(backend_path), conf=0.0, shape_only=True)
            result = raw.detect(img)

            dist = ((ref_cx - result["center"][0]) ** 2 + (ref_cy - result["center"][1]) ** 2) ** 0.5
            ok = (
                result is not None
                and result["class_name"] == ref_class
                and abs(result["confidence"] - ref_conf) < 1e-3
                and dist < CENTER_TOLERANCE_PX
            )
            all_ok &= ok
            print(
                f"  {backend_path.name:22s} ultralytics=({ref_cx:.1f},{ref_cy:.1f}) conf={ref_conf:.4f} | "
                f"raw_detector=({result['center'][0]:.1f},{result['center'][1]:.1f}) "
                f"conf={result['confidence']:.4f} | {'OK' if ok else 'MISMATCH'}"
            )

    print("\nALL MATCH" if all_ok else "\nMISMATCHES FOUND -- do not trust raw_detector.py until fixed")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
