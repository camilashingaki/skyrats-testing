#!/usr/bin/env python3
"""
Takeoff, fly 5 m forward, land -- Nectar SDK version.

Uses move_to(x=distance, reference=BODY) -- Nectar's own closed-loop
navigation primitive polls position internally and returns once within
`--precision` meters, so (unlike andar_frente.py's SkyMAVLink version) no
manual arrival-polling loop is needed here.

yaw=None keeps the heading fixed for the whole move: Nectar's PID
navigation always sends an explicit zero yaw-rate when yaw is None (see
nectar/control/mavlink/transport.py's _VELOCITY_MASK, which never flags
yaw-rate "ignore" the way it does yaw), so the drone doesn't turn to face
the target the way ArduPilot's default WP_YAW_BEHAVIOR would without it --
the same concern the SkyMAVLink library's own rules doc (CLAUDE.md)
documents for SkyMAVLink's own set_body_velocity().

Usage:
    python andar_frente_nectar.py
    python andar_frente_nectar.py --connection /dev/serial0 --baud 921600 --distance 5.0
"""

import argparse
import logging

import nectar
from nectar.control import MoveReference, NavigationMethod

from nectar_util import add_nectar_args, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly forward a fixed distance, land (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--distance", type=float, default=5.0, help="Distance to fly forward, in meters.")
    parser.add_argument("--precision", type=float, default=0.2, help="Arrival threshold in meters.")
    parser.add_argument("--timeout", type=float, default=30.0, help="Max seconds to wait for arrival.")
    parser.add_argument(
        "--nav-method", choices=["pid", "pid_ekf", "position"], default="pid",
        help="Nectar navigation method for move_to.",
    )
    args = parser.parse_args()
    method = getattr(NavigationMethod, args.nav_method.upper())

    nectar.init()
    drone = create_drone(args)
    try:
        if not drone.connect():
            log.error("Failed to connect to the vehicle")
            return
        if not drone.takeoff(altitude=args.height):
            log.error("Takeoff failed")
            return

        log.info("Flying %.1f m forward (heading locked)", args.distance)
        reached = drone.move_to(
            x=args.distance, y=0.0, z=0.0, yaw=None,
            reference=MoveReference.BODY, method=method,
            precision=args.precision, timeout=args.timeout,
        )
        if not reached:
            log.warning("Did not confirm arrival within %.0fs; landing anyway.", args.timeout)
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
