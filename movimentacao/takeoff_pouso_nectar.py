#!/usr/bin/env python3
"""
Takeoff to ~3 m, print altitude while hovering, land -- Nectar SDK version.

Same mission as takeoff_pouso.py, built on nectar.control.DroneFactory
instead of skymavlink.

Usage:
    python takeoff_pouso_nectar.py                          # SITL, mavlink backend
    python takeoff_pouso_nectar.py --connection /dev/serial0 --baud 921600
"""

import argparse
import logging

import nectar

from nectar_util import add_nectar_args, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, print altitude while hovering, land (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--altitude", type=float, default=3.0, help="Takeoff altitude in meters.")
    parser.add_argument("--hover-time", type=float, default=5.0, help="Seconds to hover before landing.")
    parser.add_argument("--print-period", type=float, default=1.0, help="Seconds between altitude prints.")
    args = parser.parse_args()

    nectar.init()
    drone = create_drone(args)
    try:
        if not drone.connect():
            log.error("Failed to connect to the vehicle")
            return
        if not drone.takeoff(altitude=args.altitude):
            log.error("Takeoff failed")
            return
        log.info("Reached %.2f m", drone.get_altitude() or 0.0)

        elapsed = 0.0
        while elapsed < args.hover_time:
            drone.delay(args.print_period)
            elapsed += args.print_period
            log.info("Altitude: %.2f m", drone.get_altitude() or 0.0)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
