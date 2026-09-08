#!/usr/bin/env python3
"""
Takeoff, fly a square, land -- Nectar SDK version.

Same mission as quadrado.py, built on nectar.control.DroneFactory. Each leg
is a move_to(..., reference=BODY, yaw=None) call. Nectar's BODY reference is
FLU (x=forward, y=left), so the four legs below read forward/left/back/right
(counter-clockwise) rather than SkyMAVLink's forward/right/back/left
(clockwise, FRD) -- either order traces the same square, just walked the
other way around.

yaw=None on every leg keeps the heading locked for the same reason as
andar_frente_nectar.py (see that script's docstring): Nectar's PID
navigation always sends an explicit zero yaw-rate rather than omitting it,
so the drone doesn't yaw toward each new corner.

Usage:
    python quadrado_nectar.py
    python quadrado_nectar.py --connection /dev/serial0 --baud 921600 --side 3.0
"""

import argparse
import logging

import nectar
from nectar.control import MoveReference, NavigationMethod

from nectar_util import add_nectar_args, create_drone, log


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square with heading locked, land (Nectar).")
    add_nectar_args(parser)
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--precision", type=float, default=0.2, help="Arrival threshold in meters.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
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

        log.info("Square complete")
    except KeyboardInterrupt:
        log.info("Interrupted -- landing")
    finally:
        drone.land()
        drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
