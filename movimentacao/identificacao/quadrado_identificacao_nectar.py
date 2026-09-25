#!/usr/bin/env python3
"""
Takeoff, fly a square (heading locked, same pattern as
movimentacao/quadrado_nectar.py), taking photos and feeding every one of
them to the NCNN base-detection model along the way, then land wherever the
square ends -- Nectar SDK version. This script only identifies -- it never
centers on or lands on top of a detected base; for that, see
../../missions/precision_landing/.

Uses nectar.ai.detection.Detector, same as
base_detection/detect_base_nectar.py: it auto-detects Ultralytics as the
framework for a NCNN export directory or a .tflite path exactly like it
does for a .pt checkpoint (ultralytics.YOLO() itself dispatches on the
path -- see base_detection/README.md), so no extra plumbing is needed to
point it at the committed NCNN/TFLite exports. This does pull in
ultralytics/torch, unlike quadrado_identificacao.py's torch-free
RawDetector -- that's the tradeoff of going through Nectar's own detection
wrapper instead of base_detection/raw_detector.py directly.

Detection runs inside ImageHandler's on_frame() callback (own background
thread), throttled to --photo-period, independent of the leg-by-leg
move_to() calls in the mission thread.

Usage:
    python quadrado_identificacao_nectar.py
    python quadrado_identificacao_nectar.py --side 3.0 --model ../../base_detection/models/best_w8a32.tflite
"""

import argparse
import logging
import os
import threading
import time

import cv2
import nectar
from nectar.ai.detection import Detector
from nectar.control import MoveReference, NavigationMethod
from nectar.vision.camera import ImageHandler

from nectar_util import DEFAULT_MODEL, add_nectar_args, build_camera_config, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square identifying the base along the way, land (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="Detector model path (best.pt, NCNN dir, or .tflite).")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum detection confidence.")
    parser.add_argument("--classes", nargs="*", default=None, help="Class names counted as a hit. Default: any detected class.")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--out-dir", default="identificacao_output", help="Directory to save photos/detections to.")
    parser.add_argument("--photo-period", type=float, default=1.0, help="Seconds between photos while flying.")
    parser.add_argument("--precision", type=float, default=0.2, help="Arrival threshold in meters.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
    parser.add_argument(
        "--nav-method", choices=["pid", "pid_ekf", "position"], default="pid",
        help="Nectar navigation method for move_to.",
    )
    args = parser.parse_args()
    method = getattr(NavigationMethod, args.nav_method.upper())
    target_classes = set(args.classes) if args.classes else None

    detector = Detector(args.model)
    detector.load()
    log.info("Loaded %s detector %s classes=%s", detector.framework, args.model, detector.class_names)

    photos_dir = os.path.join(args.out_dir, "fotos")
    hits_dir = os.path.join(args.out_dir, "deteccoes")
    os.makedirs(photos_dir, exist_ok=True)
    os.makedirs(hits_dir, exist_ok=True)
    lock = threading.Lock()
    state = {"count": 0, "hits": 0, "last": 0.0}

    def on_frame(frame) -> None:
        now = time.time()
        with lock:
            if (now - state["last"]) < args.photo_period:
                return
            state["last"] = now
            state["count"] += 1
            idx = state["count"]
        cv2.imwrite(os.path.join(photos_dir, f"foto_{idx:03d}.jpg"), frame)

        result = detector.detect(frame, conf=args.conf)
        candidates = [d for d in (result or []) if target_classes is None or d.class_name in target_classes]
        if not candidates:
            return
        best = max(candidates, key=lambda d: d.confidence)
        with lock:
            state["hits"] += 1
        log.info(
            "DETECTED foto_%03d -> class=%s conf=%.2f center=%s",
            idx, best.class_name, best.confidence, best.center,
        )
        cv2.imwrite(os.path.join(hits_dir, f"deteccao_{idx:03d}.jpg"), detector.draw_detections(frame, result))

    cam_config, cam_source = build_camera_config(args.camera_type)
    camera = ImageHandler(
        image_source=cam_source, config=cam_config, image_processing_callback=on_frame, poll_interval=0.05)
    camera.run()

    nectar.init()
    drone = create_drone(args)
    try:
        if not drone.connect():
            log.error("Failed to connect to the vehicle")
            return
        if not drone.takeoff(altitude=args.height):
            log.error("Takeoff failed")
            return

        side = args.side
        legs = [(side, 0.0), (0.0, side), (-side, 0.0), (0.0, -side)]  # (fwd, left), FLU
        for i, (fwd, left) in enumerate(legs, start=1):
            log.info("Leg %d/4: fwd=%.2f left=%.2f", i, fwd, left)
            reached = drone.move_to(
                x=fwd, y=left, z=0.0, yaw=None,
                reference=MoveReference.BODY, method=method,
                precision=args.precision, timeout=args.leg_timeout,
            )
            if not reached:
                log.warning("Leg %d did not confirm arrival within %.0fs; continuing.", i, args.leg_timeout)

        with lock:
            count, hits = state["count"], state["hits"]
        log.info("Square complete -- %d photos captured, %d hits (%s)", count, hits, args.out_dir)

        # No centering/landing-on-target here -- land wherever the square ended.
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        camera.cleanup()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
