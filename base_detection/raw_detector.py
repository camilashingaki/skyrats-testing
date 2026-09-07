"""YOLO inference for the base-detection model without torch/ultralytics.

Only works with models exported by model_export/export_model.py (NCNN or
TFLite). It relies on a fact verified against this exact checkpoint (see
model_export/README.md): the exported graph already bakes in the anchor/
stride box decode and the class-score sigmoid, so no DFL/anchor-decode math
needs to be reimplemented here -- the raw output is already
(4 box coords + N class scores) x 8400 anchors, boxes already in pixel space
relative to the padded 640x640 input, as (center_x, center_y, width, height)
-- verified by comparing against ultralytics.YOLO(...).predict() on the same
exported graph; this checkpoint's metadata reports end2end=false, which
selects xywh in ultralytics' own decode_bboxes(), not xyxy.

Only the shape classes (hexagon/star/triangle) are matched by default: a
base is a single shape with a number printed inside it, so a real base
always fires both a shape_* and a number_* detection on the same object --
matching both would count one physical base as two candidates. Matching
shape_* alone avoids that without needing to retrain or merge classes.

Output is a centroid (no bounding box): the only thing the centering loop in
detect_base_*.py ever reads from a detection is its center point.
"""

from typing import Optional

import cv2
import numpy as np

CLASS_NAMES = ["shape_hexagon", "shape_star", "shape_triangle", "number_3", "number_4", "number_5"]
SHAPE_CLASS_INDICES = (0, 1, 2)

IMG_SIZE = 640
PAD_VALUE = 114


def _letterbox(frame: np.ndarray):
    """Resize+pad to IMG_SIZE x IMG_SIZE preserving aspect ratio (ultralytics' LetterBox, auto=False)."""
    h, w = frame.shape[:2]
    ratio = min(IMG_SIZE / h, IMG_SIZE / w)
    new_w, new_h = round(w * ratio), round(h * ratio)
    dw, dh = (IMG_SIZE - new_w) / 2, (IMG_SIZE - new_h) / 2
    top, bottom = round(dh - 0.1), round(dh + 0.1)
    left, right = round(dw - 0.1), round(dw + 0.1)

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(PAD_VALUE,) * 3)
    return padded, ratio, left, top


class _NCNNBackend:
    def __init__(self, model_dir: str) -> None:
        import ncnn

        self._ncnn = ncnn
        self.net = ncnn.Net()
        self.net.load_param(f"{model_dir}/model.ncnn.param")
        self.net.load_model(f"{model_dir}/model.ncnn.bin")

    def run(self, padded_bgr: np.ndarray) -> np.ndarray:
        ncnn = self._ncnn
        mat_in = ncnn.Mat.from_pixels(padded_bgr, ncnn.Mat.PixelType.PIXEL_BGR2RGB, IMG_SIZE, IMG_SIZE)
        mat_in.substract_mean_normalize([0.0, 0.0, 0.0], [1 / 255.0, 1 / 255.0, 1 / 255.0])
        ex = self.net.create_extractor()
        ex.input("in0", mat_in)
        _, mat_out = ex.extract("out0")
        return np.array(mat_out)  # (10, 8400)


class _TFLiteBackend:
    def __init__(self, model_path: str) -> None:
        from ai_edge_litert.interpreter import Interpreter

        self.interpreter = Interpreter(model_path)
        self.interpreter.allocate_tensors()
        self._input_index = self.interpreter.get_input_details()[0]["index"]
        self._output_index = self.interpreter.get_output_details()[0]["index"]

    def run(self, padded_bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(padded_bgr, cv2.COLOR_BGR2RGB)
        chw = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        input_tensor = np.ascontiguousarray(chw[None])  # (1, 3, 640, 640)

        self.interpreter.set_tensor(self._input_index, input_tensor)
        self.interpreter.invoke()
        out = self.interpreter.get_tensor(self._output_index)[0].copy()  # (10, 8400)
        out[:4] *= IMG_SIZE  # LiteRT normalizes box coords to [0, 1]; NCNN/PyTorch keep them in pixels
        return out


class RawDetector:
    """Drop-in replacement for the ultralytics-backed Detector, no torch required."""

    def __init__(self, model_path: str, conf: float = 0.5, shape_only: bool = True) -> None:
        self.conf = conf
        self.class_indices = list(SHAPE_CLASS_INDICES if shape_only else range(len(CLASS_NAMES)))
        self._backend = _TFLiteBackend(model_path) if str(model_path).endswith(".tflite") else _NCNNBackend(str(model_path))

    @property
    def class_names(self):
        return CLASS_NAMES

    def detect(self, frame: np.ndarray, conf: Optional[float] = None) -> Optional[dict]:
        """Return the single highest-confidence match, or None. Mirrors best_candidate() in detect_base_*.py."""
        threshold = self.conf if conf is None else conf
        padded, ratio, pad_x, pad_y = _letterbox(frame)
        raw = self._backend.run(padded)  # (10, 8400): rows 0-3 = cx,cy,w,h (px, padded space); 4.. = class scores

        boxes, scores = raw[:4], raw[4:]
        cls_scores = scores[self.class_indices]  # (len(class_indices), 8400)
        best_local_cls, best_anchor = np.unravel_index(np.argmax(cls_scores), cls_scores.shape)
        best_conf = float(cls_scores[best_local_cls, best_anchor])
        if best_conf < threshold:
            return None

        box_cx, box_cy, _w, _h = boxes[:, best_anchor]
        cx = (box_cx - pad_x) / ratio
        cy = (box_cy - pad_y) / ratio

        return {
            "class_name": CLASS_NAMES[self.class_indices[best_local_cls]],
            "confidence": best_conf,
            "center": (float(cx), float(cy)),
        }

    def draw(self, frame: np.ndarray, result: Optional[dict], radius: int = 10, color=(0, 0, 255)) -> np.ndarray:
        """Mark the detected centroid (no box, since none is computed)."""
        annotated = frame.copy()
        if result is None:
            return annotated
        cx, cy = (int(round(v)) for v in result["center"])
        cv2.drawMarker(annotated, (cx, cy), color, markerType=cv2.MARKER_CROSS, markerSize=2 * radius, thickness=2)
        cv2.circle(annotated, (cx, cy), radius, color, 2)
        label = f'{result["class_name"]} {result["confidence"]:.2f}'
        cv2.putText(annotated, label, (cx + radius + 4, cy - radius), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return annotated
