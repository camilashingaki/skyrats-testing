"""Shared Nectar SDK (Black-Bee-Drones/nectar-sdk) setup for the
movimentacao/ *_nectar.py scripts.

build_drone_config() mirrors base_detection/detect_base_nectar.py's own
copy of the same function -- duplicated rather than imported, since that
script is a standalone mission, not a library, same as every other file in
this repo that touches Nectar.
"""

import argparse
import logging

from nectar.control import (
    DroneFactory,
    MavlinkConfig,
    MavrosConfig,
    PoseSource,
    Px4DdsConfig,
    Px4MavlinkConfig,
    Px4MavrosConfig,
)

log = logging.getLogger("movimentacao_nectar")

# SITL default (Nectar's "mavlink" backend talks pymavlink directly, so this
# is the same tcp:host:port form used elsewhere in this repo) -- an
# accidental run never moves a real drone.
ENDPOINT_SITL = "tcp:127.0.0.1:5762"


def add_nectar_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--drone", choices=["mavlink", "mavros", "px4", "px4_mavlink", "px4_dds"], default="mavlink",
        help="Nectar drone backend to use.",
    )
    parser.add_argument("--env", choices=["outdoor", "indoor"], default="outdoor")
    parser.add_argument(
        "--connection", default=ENDPOINT_SITL,
        help="Serial device (mavlink/px4_mavlink) or fcu_url (mavros/px4). Default: SITL. "
        "Real hardware: /dev/serial0 (the Pi's UART pins).",
    )
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate; must match the FC's SERIALx_BAUD for this port.")


def build_drone_config(args: argparse.Namespace):
    pose_source = PoseSource.VISION if args.env == "indoor" else PoseSource.GPS
    kwargs = {"pose_source": pose_source, "start_driver": False}

    if args.drone in ("mavros", "px4"):
        # MAVROS embeds the baud rate in the connection URL itself.
        if args.connection:
            connection_string = args.connection
            if connection_string.startswith("/dev/"):
                connection_string = f"serial://{connection_string}:{args.baud}"
            kwargs["connection_string"] = connection_string
    else:
        # Direct-pymavlink backends (mavlink/px4_mavlink) take a bare device
        # path plus a separate baud field.
        if args.connection:
            kwargs["connection_string"] = args.connection
        kwargs["baud"] = args.baud

    if args.drone == "mavros":
        return MavrosConfig(**kwargs)
    if args.drone == "mavlink":
        return MavlinkConfig(**kwargs)
    if args.drone == "px4":
        return Px4MavrosConfig(**kwargs)
    if args.drone == "px4_mavlink":
        return Px4MavlinkConfig(**kwargs)
    if args.drone == "px4_dds":
        return Px4DdsConfig(pose_source=pose_source, start_driver=False)
    raise ValueError(f"Unsupported --drone for this mission: {args.drone}")


def create_drone(args: argparse.Namespace):
    return DroneFactory.create(args.drone, build_drone_config(args))
