"""Shared Nectar SDK (Black-Bee-Drones/nectar-sdk) setup for
quadrado_identificacao_nectar.py.

build_drone_config() and build_camera_config() mirror
base_detection/detect_base_nectar.py's own copies -- duplicated rather than
imported, since that script is a standalone mission, not a library, same as
every other file in this repo that touches Nectar.
"""

import argparse
import logging
from pathlib import Path

from nectar.control import (
    DroneFactory,
    MavlinkConfig,
    MavrosConfig,
    PoseSource,
    Px4DdsConfig,
    Px4MavlinkConfig,
    Px4MavrosConfig,
)
from nectar.vision.camera.config_builder import ConfigBuilder

log = logging.getLogger("identificacao_nectar")

# SITL default -- an accidental run never moves a real drone.
ENDPOINT_SITL = "tcp:127.0.0.1:5762"

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = REPO_ROOT / "base_detection" / "models" / "best_ncnn_model"

_CAMERA_PARAMS = {
    "webcam": {"device_index": 0, "width": 1280, "height": 720, "fps": 30},
    "imx219": {"sensor_id": 0, "width": 1280, "height": 720, "flip": 2},
    "ros": {"topic": "/camera/color/image_raw/compressed", "compressed": True},
}


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
    parser.add_argument("--camera-type", default="webcam", help="webcam | imx219 | ros | <video file path> | <ROS topic>")


def build_drone_config(args: argparse.Namespace):
    pose_source = PoseSource.VISION if args.env == "indoor" else PoseSource.GPS
    kwargs = {"pose_source": pose_source, "start_driver": False}

    if args.drone in ("mavros", "px4"):
        if args.connection:
            connection_string = args.connection
            if connection_string.startswith("/dev/"):
                connection_string = f"serial://{connection_string}:{args.baud}"
            kwargs["connection_string"] = connection_string
    else:
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


def build_camera_config(camera_type: str):
    """Return (config, source_key) for ImageHandler."""
    params = _CAMERA_PARAMS.get(camera_type)
    if params is None:
        return None, camera_type
    return ConfigBuilder.build(camera_type, params), camera_type
