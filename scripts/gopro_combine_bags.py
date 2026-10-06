#!/usr/bin/env python3
"""Combines a left and a right GoPro bag (written by gopro_ros) into one stereo bag, optionally
with a center GoPro bag as a third camera.

Works with ROS 1 bags and ROS 2 bags (MCAP or SQLite3); the ROS version is taken from $ROS_VERSION.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ros_compat import ROS2_STORAGE_IDS, ROS_VERSION, BagReader, BagWriter  # noqa: E402

# gopro_ros topic -> topic name in the combined bag, below /gopro/left, /gopro/right or /gopro/center
TOPICS = {
    "/gopro/image_raw": "image_raw",
    "/gopro/image_raw/compressed": "image_raw/compressed",
    "/gopro/imu": "imu",
    "/gopro/magnetic_field": "magnetic_field",
}


def copy_bag(reader: BagReader, writer: BagWriter, side: str):
    types = reader.topic_types()
    for topic, msg, t in reader:
        writer.write(f"/gopro/{side}/{TOPICS[topic]}", msg, t, types[topic])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="combine_gopro_bags",
        description="Combine left and right (and optionally center) gopro bags into one bag",
        add_help=True,
    )
    parser.add_argument("--left_bag", "-l", type=str, help="path to left gopro bag")
    parser.add_argument("--right_bag", "-r", type=str, help="path to right gopro bag")
    parser.add_argument("--center_bag", "-c", type=str, help="path to center gopro bag (optional)")
    parser.add_argument("--output_bag", "-o", type=str, help="path to output bag")
    parser.add_argument(
        "--storage_id",
        "-s",
        choices=list(ROS2_STORAGE_IDS),
        default=".mcap",
        help="ROS 2 only: storage of the output bag (default: .mcap)",
    )

    args = parser.parse_args()
    if args.left_bag is None or args.right_bag is None or args.output_bag is None:
        print("Please specify all bag paths")
        exit(1)

    storage = f" ({args.storage_id})" if ROS_VERSION == 2 else ""
    print(f"Writing ROS {ROS_VERSION} bag {args.output_bag}{storage}")

    writer = BagWriter(args.output_bag, args.storage_id)
    bags = [("left", args.left_bag), ("right", args.right_bag)]
    if args.center_bag is not None:
        bags.append(("center", args.center_bag))
    for side, path in bags:
        # On ROS 2 the messages are copied without deserializing them
        reader = BagReader(path, topics=TOPICS, deserialize=False)
        copy_bag(reader, writer, side)
        reader.close()
    writer.close()
