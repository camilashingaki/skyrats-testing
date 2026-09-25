#!/usr/bin/env python3
"""
Takeoff and hover, showing the live camera feed in a window until 'q' is
pressed or a timeout is reached, then land -- Nectar SDK version.

Uses nectar.vision.camera.ImageHandler for capture (same setup as
base_detection/detect_base_nectar.py) instead of raw OpenCV VideoCapture:
ImageHandler runs its own background thread and hands each frame to
on_frame() below, which just stashes the latest one; the main loop displays
whatever's newest and services the drone's delay() between draws.

Requires a display (opencv-python with GUI support, not
opencv-python-headless).

Usage:
    python camera_live_view_nectar.py                          # SITL, mavlink backend
    python camera_live_view_nectar.py --connection /dev/serial0 --baud 921600 --camera-type imx219
"""

import argparse
import logging
import threading
import time

import cv2
import nectar
from nectar.vision.camera import ImageHandler

from nectar_util import add_nectar_args, build_camera_config, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff and show the live camera feed (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--max-time", type=float, default=120.0, help="Max seconds to hover/view before landing anyway.")
    args = parser.parse_args()

    frame_lock = threading.Lock()
    latest_frame = [None]

    def on_frame(frame) -> None:
        with frame_lock:
            latest_frame[0] = frame

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

        log.info("Hovering at %.1f m -- showing live feed, press 'q' to land.", args.height)
        start = time.time()
        while time.time() - start < args.max_time:
            with frame_lock:
                frame = latest_frame[0]
            if frame is not None:
                cv2.imshow("Nectar live view", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                log.info("Quit requested -- landing.")
                break
            drone.delay(0.01)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        camera.cleanup()
        drone.cleanup()
        nectar.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
