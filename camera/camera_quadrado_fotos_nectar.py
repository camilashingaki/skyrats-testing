#!/usr/bin/env python3
"""
Takeoff, fly a square (heading locked, same pattern as
movimentacao/quadrado_nectar.py), saving a photo on a fixed cadence along
the way, land -- Nectar SDK version.

move_to() blocks for the whole leg with no per-tick hook, so photo capture
here doesn't ride along a leg-progress loop like the SkyMAVLink version does
-- it's throttled directly inside ImageHandler's on_frame() callback, which
fires continuously on its own background thread for as long as the camera
runs, independent of what the mission thread is doing.

Usage:
    python camera_quadrado_fotos_nectar.py
    python camera_quadrado_fotos_nectar.py --side 3.0 --photo-period 0.5 --out-dir fotos
"""

import argparse
import logging
import os
import threading
import time

import cv2
import nectar
from nectar.control import MoveReference, NavigationMethod
from nectar.vision.camera import ImageHandler

from nectar_util import add_nectar_args, build_camera_config, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square, taking photos, land (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--out-dir", default="fotos", help="Directory to save photos to.")
    parser.add_argument("--photo-period", type=float, default=1.0, help="Seconds between photos while flying.")
    parser.add_argument("--precision", type=float, default=0.2, help="Arrival threshold in meters.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
    parser.add_argument(
        "--nav-method", choices=["pid", "pid_ekf", "position"], default="pid",
        help="Nectar navigation method for move_to.",
    )
    args = parser.parse_args()
    method = getattr(NavigationMethod, args.nav_method.upper())

    os.makedirs(args.out_dir, exist_ok=True)
    lock = threading.Lock()
    state = {"count": 0, "last_saved": 0.0}

    def on_frame(frame) -> None:
        now = time.time()
        with lock:
            if (now - state["last_saved"]) < args.photo_period:
                return
            state["last_saved"] = now
            state["count"] += 1
            idx = state["count"]
        path = os.path.join(args.out_dir, f"foto_{idx:03d}.jpg")
        cv2.imwrite(path, frame)
        log.info("Saved %s", path)

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
            count = state["count"]
        log.info("Square complete -- %d photos saved to %s", count, args.out_dir)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        camera.cleanup()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
