#!/usr/bin/env python3
"""
Takeoff, fly a square (heading locked, same pattern as
movimentacao/quadrado.py), taking photos on a fixed cadence along the way,
land.

Usage:
    python camera_quadrado_fotos.py
    python camera_quadrado_fotos.py --side 3.0 --photo-period 0.5 --out-dir fotos
"""

import argparse
import logging
import math

import cv2

from skymavlink import SkyMAVLink

from _util import ENDPOINT_SITL, PhotoSaver, current_heading_deg, log, wait_until_arrived


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square, taking photos, land.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera device index.")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--out-dir", default="fotos", help="Directory to save photos to.")
    parser.add_argument("--photo-period", type=float, default=1.0, help="Seconds between photos while flying.")
    parser.add_argument("--arrive-tolerance", type=float, default=0.2, help="Meters from target counted as arrived.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        log.error("Failed to open camera index %d", args.camera_index)
        return
    saver = PhotoSaver(cap, args.out_dir, period=args.photo_period)

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.height)
        saver.maybe_capture(force=True)  # one shot right after reaching altitude

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
                on_tick=saver.maybe_capture,
            ):
                log.warning("Leg %d did not confirm arrival within %.0fs; continuing.", i, args.leg_timeout)
            saver.maybe_capture(force=True)  # one at each corner

        log.info("Square complete -- %d photos saved to %s", saver.count, args.out_dir)
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
