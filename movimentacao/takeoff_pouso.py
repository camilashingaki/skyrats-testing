#!/usr/bin/env python3
"""
Takeoff to ~3 m, print the altitude while hovering, land.

The simplest possible SkyMAVLink mission: no lateral movement at all. Useful
as the first smoke test on a new vehicle/link before trying anything that
moves horizontally.

Usage:
    python takeoff_pouso.py                       # SITL, tcp:127.0.0.1:5760
    python takeoff_pouso.py --connection serial:/dev/ttyACM0:115200 --altitude 3.0
"""

import argparse
import logging

from skymavlink import SkyMAVLink

from _util import ENDPOINT_SITL, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, print altitude while hovering, land.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--altitude", type=float, default=3.0, help="Takeoff altitude in meters.")
    parser.add_argument("--hover-time", type=float, default=5.0, help="Seconds to hover before landing.")
    parser.add_argument("--print-period", type=float, default=1.0, help="Seconds between altitude prints.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")
    args = parser.parse_args()

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.altitude)
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.altitude)

        pose = drone.get_pose_local()
        log.info("Reached %.2f m", -pose[2] if pose else float("nan"))

        elapsed = 0.0
        while elapsed < args.hover_time:
            drone.sleep(args.print_period)
            elapsed += args.print_period
            pose = drone.get_pose_local()
            altitude = -pose[2] if pose is not None else float("nan")
            log.info("Altitude: %.2f m", altitude)

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


if __name__ == "__main__":
    main()
