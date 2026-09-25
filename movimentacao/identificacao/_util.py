"""Shared helpers for the movimentacao/identificacao/ mission script.

Reuses base_detection's torch-free RawDetector (NCNN/TFLite) instead of
reimplementing YOLO inference -- see ../../base_detection/README.md and
raw_detector.py for how that model was exported and how it decodes.
"""

import logging
import math
import os
import sys
import time
from pathlib import Path

import cv2

log = logging.getLogger("identificacao")

ENDPOINT_SITL = "tcp:127.0.0.1:5760"

REPO_ROOT = Path(__file__).resolve().parents[2]
BASE_DETECTION_DIR = REPO_ROOT / "base_detection"
DEFAULT_MODEL = BASE_DETECTION_DIR / "models" / "best_ncnn_model"

# base_detection/ has no __init__.py -- it's a folder of standalone scripts,
# not a package -- so raw_detector is imported by adding its directory to
# sys.path rather than a package-relative import.
if str(BASE_DETECTION_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DETECTION_DIR))

from raw_detector import RawDetector  # noqa: E402


def current_heading_deg(drone) -> float:
    """Current yaw in degrees CW from North, or 0.0 if attitude hasn't arrived yet."""
    att = drone.get_attitude()
    return math.degrees(att[2]) if att is not None else 0.0


def wait_until_arrived(drone, target_n, target_e, tolerance=0.2, timeout=30.0, poll_period=0.2, on_tick=None):
    """Poll get_pose_local() until within `tolerance` meters of the target.

    Calls on_tick() once per poll, if given -- used here to capture a photo
    and run it through the detector on a fixed cadence while a leg is in
    flight, without a second thread.
    """
    elapsed = 0.0
    while elapsed < timeout:
        drone.sleep(poll_period)
        elapsed += poll_period
        if on_tick is not None:
            on_tick()
        pose = drone.get_pose_local()
        if pose is None:
            continue
        n, e, _d = pose
        if math.hypot(target_n - n, target_e - e) <= tolerance:
            return True
    return False


class DetectingCamera:
    """Captures frames on a fixed cadence, saves each one, and runs it
    through a RawDetector -- saving an annotated copy and logging a hit
    whenever the model finds something.

    Every frame is fed to the model ("ja jogar essas fotos para o modelo
    ncnn"): this isn't just a photo log, detection runs inline on each
    captured photo.
    """

    def __init__(
        self,
        cap,
        detector: RawDetector,
        out_dir: str,
        period: float = 1.0,
        target_classes=None,
    ) -> None:
        self.cap = cap
        self.detector = detector
        self.target_classes = target_classes
        self.period = period
        self._last_capture = 0.0
        self._count = 0
        self.hits = []  # list of (photo_index, class_name, confidence, center)

        self.photos_dir = os.path.join(out_dir, "fotos")
        self.hits_dir = os.path.join(out_dir, "deteccoes")
        os.makedirs(self.photos_dir, exist_ok=True)
        os.makedirs(self.hits_dir, exist_ok=True)

    @property
    def count(self) -> int:
        return self._count

    def maybe_capture(self, force: bool = False):
        """Grab a frame, save it, and run it through the detector.

        Returns the detection dict if the model found a match, else None.
        """
        now = time.time()
        if not force and (now - self._last_capture) < self.period:
            return None
        ok, frame = self.cap.read()
        if not ok:
            return None
        self._last_capture = now
        self._count += 1

        cv2.imwrite(os.path.join(self.photos_dir, f"foto_{self._count:03d}.jpg"), frame)

        result = self.detector.detect(frame)
        if result is None or (
            self.target_classes is not None and result["class_name"] not in self.target_classes
        ):
            return None

        self.hits.append((self._count, result["class_name"], result["confidence"], result["center"]))
        log.info(
            "DETECTED foto_%03d -> class=%s conf=%.2f center=%s",
            self._count, result["class_name"], result["confidence"], result["center"],
        )
        annotated = self.detector.draw(frame, result)
        cv2.imwrite(os.path.join(self.hits_dir, f"deteccao_{self._count:03d}.jpg"), annotated)
        return result
