#!/usr/bin/env python3
"""
Takeoff, fly a square (body-frame legs: forward, right, back, left), land.

Heading is locked to whatever it was at takeoff for the entire mission: every
leg is a set_body_pose() call with an explicit yaw_deg, so the drone strafes
around the square nose-first-fixed instead of yawing to face each new
waypoint (ArduPilot's default WP_YAW_BEHAVIOR would otherwise turn the nose
toward the direction of travel on every leg -- see the SkyMAVLink library's
own rules doc (CLAUDE.md), "Never flag yaw_rate ignore on a velocity
setpoint" -- the same effect shows up on unyawed position targets, not just
velocity ones).

Usage:
    python quadrado.py                          # SITL, tcp:127.0.0.1:5760, 2 m side
    python quadrado.py --connection serial:/dev/ttyACM0:115200 --side 3.0
"""

import argparse
import logging
import math

from skymavlink import SkyMAVLink

from _util import ENDPOINT_SITL, current_heading_deg, log, wait_until_arrived


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly a square with heading locked, land.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=2.0, help="Square side length, in meters.")
    parser.add_argument("--arrive-tolerance", type=float, default=0.2, help="Meters from target counted as arrived.")
    parser.add_argument("--leg-timeout", type=float, default=20.0, help="Max seconds to wait per leg.")
    parser.add_argument("--arm-timeout", type=float, default=15.0, help="Seconds to wait for arm confirmation.")
    args = parser.parse_args()

    drone = SkyMAVLink(args.connection, takeoff_altitude=args.height)
    try:
        drone.wait_for_connection()
        drone.set_mode("GUIDED")
        drone.arm(timeout=args.arm_timeout)
        drone.takeoff(args.height)

        north, east, _down = drone.wait_for_position()
        yaw_deg = current_heading_deg(drone)
        yaw_rad = math.radians(yaw_deg)
        log.info("Heading locked at %.1f deg for the whole square", yaw_deg)

        # Body-frame (fwd, right) offsets for each leg: forward, right, back, left.
        # Rotated into NED once up front using the locked heading, so arrival can
        # be checked against a fixed target rather than a moving one.
        side = args.side
        legs_frd = [(side, 0.0), (0.0, side), (-side, 0.0), (0.0, -side)]
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)

        pos_n, pos_e = north, east
        for i, (fwd, right) in enumerate(legs_frd, start=1):
            pos_n += fwd * c + right * (-s)
            pos_e += fwd * s + right * c
            log.info("Leg %d/4: target N=%.2f E=%.2f", i, pos_n, pos_e)
            drone.set_body_pose(fwd, right, 0.0, yaw_deg=yaw_deg)
            if not wait_until_arrived(drone, pos_n, pos_e, args.arrive_tolerance, args.leg_timeout):
                log.warning("Leg %d did not confirm arrival within %.0fs; continuing.", i, args.leg_timeout)

        log.info("Square complete")
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
