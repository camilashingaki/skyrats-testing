#!/usr/bin/env python3
"""
Takeoff and hover, showing the live camera feed in a window until 'q' is
pressed or a timeout is reached, then land.

Requires a display (opencv-python with GUI support, not opencv-python-headless)
-- run it on a companion computer with HDMI/VNC, or over X11 forwarding.

Usage:
    python camera_live_view.py
    python camera_live_view.py --connection serial:/dev/ttyACM0:115200 --height 1.5
"""

import argparse
import logging
import time

import cv2

from skymavlink import SkyMAVLink

from _util import ENDPOINT_SITL, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff and show the live camera feed.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera device index.")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--max-time", type=float, default=120.0, help="Max seconds to hover/view before landing anyway.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        log.error("Failed to open camera index %d", args.camera_index)
        return

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.height)

        log.info("Hovering at %.1f m -- showing live feed, press 'q' to land.", args.height)
        start = time.time()
        while time.time() - start < args.max_time:
            ok, frame = cap.read()
            if ok:
                cv2.imshow("SkyMAVLink live view", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                log.info("Quit requested -- landing.")
                break
            drone.sleep(0.01)  # services the MAVLink link between frames

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
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
