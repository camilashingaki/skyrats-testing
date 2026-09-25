#!/usr/bin/env python3
"""
Takeoff, fly a square (heading locked, same pattern as
movimentacao/quadrado.py), taking photos and feeding every one of them to
the NCNN base-detection model along the way, then land wherever the square
ends -- this script does not center or land on the base, see
../../missions/precision_landing/ for that.

Uses base_detection's torch-free RawDetector (NCNN by default, also accepts
a .tflite path via --model) instead of the ultralytics-based Detector in
base_detection/detect_base_*.py -- no torch/ultralytics dependency needed
here, just ncnn (or ai-edge-litert for TFLite) + opencv.

Usage:
    python quadrado_identificacao.py
    python quadrado_identificacao.py --side 3.0 --model ../../base_detection/models/best_w8a32.tflite
"""

import argparse
import logging
import math

import cv2

from skymavlink import SkyMAVLink

from _util import DEFAULT_MODEL, ENDPOINT_SITL, DetectingCamera, RawDetector, current_heading_deg, log, wait_until_arrived


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square identifying the base along the way, land.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera device index.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="RawDetector model path (NCNN dir or .tflite file).")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum detection confidence.")
    parser.add_argument(
        "--classes", nargs="*", default=None,
        help="Class names counted as a hit (e.g. shape_hexagon shape_star shape_triangle). Default: any shape class.",
    )
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--out-dir", default="identificacao_output", help="Directory to save photos/detections to.")
    parser.add_argument("--photo-period", type=float, default=1.0, help="Seconds between photos while flying.")
    parser.add_argument("--arrive-tolerance", type=float, default=0.2, help="Meters from target counted as arrived.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")
    args = parser.parse_args()

    target_classes = set(args.classes) if args.classes else None

    detector = RawDetector(args.model, conf=args.conf)
    log.info("Loaded detector %s, classes=%s", args.model, detector.class_names)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        log.error("Failed to open camera index %d", args.camera_index)
        return
    camera = DetectingCamera(cap, detector, args.out_dir, period=args.photo_period, target_classes=target_classes)

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.height)
        camera.maybe_capture(force=True)  # one shot right after reaching altitude

        north, east, _down = drone.wait_for_position()
        yaw_deg = current_heading_deg(drone)
        yaw_rad = math.radians(yaw_deg)
        log.info("Heading locked at %.1f deg for the whole square", yaw_deg)

        side = args.side
        legs_frd = [(side, 0.0), (0.0, side), (-side, 0.0), (0.0, -side)]
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)

        pos_n, pos_e = north, east
        for i, (fwd, right) in enumerate(legs_frd, start=1):
            pos_n += fwd * c + right * (-s)
            pos_e += fwd * s + right * c
            log.info("Leg %d/4: target N=%.2f E=%.2f", i, pos_n, pos_e)
            drone.set_body_pose(fwd, right, 0.0, yaw_deg=yaw_deg)
            if not wait_until_arrived(
                drone, pos_n, pos_e, args.arrive_tolerance, args.leg_timeout,
                on_tick=camera.maybe_capture,
            ):
                log.warning("Leg %d did not confirm arrival within %.0fs; continuing.", i, args.leg_timeout)
            camera.maybe_capture(force=True)  # one at each corner

        log.info(
            "Square complete -- %d photos captured, %d hits (%s)",
            camera.count, len(camera.hits), args.out_dir,
        )
        for idx, class_name, conf, center in camera.hits:
            log.info("  foto_%03d: %s conf=%.2f center=%s", idx, class_name, conf, center)

        # No centering/landing-on-target here -- land wherever the square ended.
        drone.land()
    except TimeoutError as e:
        log.error("Timeout during mission: %s", e)
        drone.rtl()
    except KeyboardInterrupt:
        log.info("Interrupted -- returning to launch")
        drone.rtl()
    finally:
        if drone.is_armed():
            drone.land()
        drone.shutdown()
        cap.release()


if __name__ == "__main__":
    main()
