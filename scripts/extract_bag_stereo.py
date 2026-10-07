#!/usr/bin/env python3
"""Writes synchronized stereo images from a bag at a fixed rate, optionally undistorted.

Works with ROS 1 bags and ROS 2 bags (MCAP or SQLite3); the ROS version is taken from $ROS_VERSION.
"""

import os
import sys

import cv2
import numpy as np
from cv_bridge import CvBridge
from tqdm import tqdm

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ros_compat import (  # noqa: E402
    BagReader,
    ExactTimeSynchronizer,
    Node,
    is_compressed_image,
    message_type,
    stamp_to_nsec,
)

cv_bridge = CvBridge()


def read_camera_intrinsics(cv_node: cv2.FileNode):  # type: ignore
    # Read camera intrinsics from cv_node
    camera_matrix = cv_node.getNode("K").mat()
    distortion_coefficients = cv_node.getNode("D").mat()

    print("Camera Matrix: ", camera_matrix)
    print("Distortion Coefficients: ", distortion_coefficients)

    return camera_matrix, distortion_coefficients


def to_cv_image(img_msg, side: str):
    if is_compressed_image(img_msg):
        return cv_bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    if message_type(img_msg) == "Image":
        return cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    raise ValueError("Unknown {} image type".format(side))


class ColmapStereo:
    def __init__(
        self,
        node: Node,
        input_bag: str,
        root_folder: str,
        left_topic: str,
        right_topic: str,
        delay: float,
        scale=1.0,
        config_file=None,
        undistort=False,
    ):
        self.node = node
        self.input_bag = input_bag
        self.scale = scale
        self.undistort = undistort

        image_folder = os.path.join(root_folder, "images")
        self.left_img_folder = os.path.join(image_folder, "left")
        self.right_img_folder = os.path.join(image_folder, "right")
        os.makedirs(self.left_img_folder, exist_ok=True)
        os.makedirs(self.right_img_folder, exist_ok=True)

        self.last_timestamp = -1
        self.delay = delay
        self.indx = 0
        self.logged_first_callback = False

        opencv_config = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)  # type: ignore
        left_K, left_dist = read_camera_intrinsics(opencv_config.getNode("left"))
        right_K, right_dist = read_camera_intrinsics(opencv_config.getNode("right"))

        height = int(opencv_config.getNode("image_height").real())
        width = int(opencv_config.getNode("image_width").real())
        print(f"Width: {width}, Height: {height}")

        self.size = (width, height)

        self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(  # type: ignore
            left_K,
            left_dist,
            np.eye(3),
            left_K,
            self.size,
            cv2.CV_32FC1,  # type: ignore
        )
        self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(  # type: ignore
            right_K,
            right_dist,
            np.eye(3),
            right_K,
            self.size,
            cv2.CV_32FC1,  # type: ignore
        )

        self.topics = [left_topic, right_topic]
        print(self.topics)

    def run(self):
        stereo_filter = ExactTimeSynchronizer(len(self.topics), 100, self.stereo_callback)

        reader = BagReader(self.input_bag, topics=self.topics)
        for topic, msg, t in tqdm(reader, total=reader.message_count()):  # type: ignore
            if not self.node.ok():
                break
            stereo_filter.add(self.topics.index(topic), msg)
        reader.close()

    def write_image(self, image, map_x, map_y, folder: str, timestamp: int):
        if self.undistort:
            image = cv2.remap(image, map_x, map_y, interpolation=cv2.INTER_LINEAR)  # type: ignore

        if self.scale != 1.0:
            image = cv2.resize(  # type: ignore
                image, (int(self.size[0] * self.scale), int(self.size[1] * self.scale))
            )

        cv2.imwrite(os.path.join(folder, "{}.png".format(str(timestamp))), image)  # type: ignore

    def stereo_callback(self, left_img_msg, right_img_msg):
        if not self.logged_first_callback:
            self.node.loginfo("Stereo callback called")
            self.logged_first_callback = True
        left_timestamp = stamp_to_nsec(left_img_msg.header.stamp)
        right_timestamp = stamp_to_nsec(right_img_msg.header.stamp)

        assert left_timestamp == right_timestamp
        if ((left_timestamp - self.last_timestamp) * 1e-9) >= self.delay:
            left_image = to_cv_image(left_img_msg, "left")
            self.write_image(
                left_image, self.left_map_x, self.left_map_y, self.left_img_folder, left_timestamp
            )

            right_image = to_cv_image(right_img_msg, "right")
            self.write_image(
                right_image,
                self.right_map_x,
                self.right_map_y,
                self.right_img_folder,
                right_timestamp,
            )

            self.last_timestamp = left_timestamp
            self.indx += 1


if __name__ == "__main__":
    node = Node("bag_extract_stereo")

    required = {
        "input_bag": "Require the input bag file",
        "image_dir": "Require the dataset path of asl directory",
        "config_file": "Require the config file path",
        "left_topic": "Require the left image topic",
        "right_topic": "Require the right image topic",
    }
    params = {name: node.get_param(name, "") for name in required}
    for name, message in required.items():
        if not params[name]:
            node.logfatal(message)
            exit(1)

    scale = node.get_param("scale", 1.0)
    freq = node.get_param("freq", 2.0)
    undistort = node.get_param("undistort", False)

    delay = 1.0 / freq  # type: ignore
    node.loginfo("Delay : {}".format(delay))

    colmap_stereo = ColmapStereo(
        node,
        params["input_bag"],
        params["image_dir"],
        params["left_topic"],
        params["right_topic"],
        delay,
        scale=scale,
        config_file=params["config_file"],
        undistort=undistort,  # type: ignore
    )
    try:
        colmap_stereo.run()
    except KeyboardInterrupt:
        print("Interrupted")
    node.shutdown()
