#!/usr/bin/env python3
"""
Takeoff, fly 5 m forward (body-frame), land.

Uses set_body_pose(), a single absolute position command computed once from
the position/heading at takeoff -- not set_body_velocity() + speed * time,
which the SkyMAVLink rules explicitly warn against ("distances come from
get_pose_local(), never from speed x time"). Heading is passed explicitly on
every call so the drone doesn't yaw toward the target (ArduPilot's default
WP_YAW_BEHAVIOR) while flying the leg.

Usage:
    python andar_frente.py                          # SITL, tcp:127.0.0.1:5760
    python andar_frente.py --connection serial:/dev/ttyACM0:115200 --distance 5.0
"""

import argparse
import logging
import math

from skymavlink import SkyMAVLink

from _util import ENDPOINT_SITL, current_heading_deg, log, wait_until_arrived


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    parser = argparse.ArgumentParser(description="Takeoff, fly forward a fixed distance, land.")
    parser.add_argument("--connection", default=ENDPOINT_SITL, help="SkyMAVLink endpoint (default: SITL).")
    parser.add_argument("--height", type=float, default=1.5, help="Takeoff altitude in meters.")
    parser.add_argument("--distance", type=float, default=5.0, help="Distance to fly forward, in meters.")
    parser.add_argument("--arrive-tolerance", type=float, default=0.2, help="Meters from target counted as arrived.")
    parser.add_argument("--timeout", type=float, default=30.0, help="Max seconds to wait for arrival.")
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
        target_n = north + args.distance * math.cos(yaw_rad)
        target_e = east + args.distance * math.sin(yaw_rad)

        log.info(
            "Flying %.1f m forward (heading locked at %.1f deg): target N=%.2f E=%.2f",
            args.distance, yaw_deg, target_n, target_e,
        )
        drone.set_body_pose(args.distance, 0.0, 0.0, yaw_deg=yaw_deg)

        if not wait_until_arrived(drone, target_n, target_e, args.arrive_tolerance, args.timeout):
            log.warning("Did not confirm arrival within %.0fs; landing anyway.", args.timeout)

        pose = drone.get_pose_local()
        if pose is not None:
            log.info("Final position N=%.2f E=%.2f", pose[0], pose[1])

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
