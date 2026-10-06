#!/usr/bin/env python3
"""Writes the images of a bag at a fixed rate, mono or stereo, as grayscale PNGs.

Works with ROS 1 bags and ROS 2 bags (MCAP or SQLite3); the ROS version is taken from $ROS_VERSION.
"""

import os
import sys

import cv2
from cv_bridge import CvBridge
from tqdm import tqdm

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ros_compat import (  # noqa: E402
    BagReader,
    ExactTimeSynchronizer,
    Node,
    is_compressed_image,
    stamp_to_nsec,
)

clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(6, 6))


class ImageExtractor:
    def __init__(
        self,
        node: Node,
        image_dir: str,
        bag_file: str,
        stereo: bool,
        compressed: bool,
        left_topic: str,
        cache_size: int = 1000,
        scale: float = 1.0,
        right_topic: str = None,
        start_time: float = 0.0,
        end_time: float = 0.0,
        display: bool = False,
    ) -> None:
        self.node = node
        self.stereo = stereo
        self.bag = bag_file
        self.img_dir = image_dir
        self.left_image_dir = os.path.join(self.img_dir, "left")
        self.right_image_dir = os.path.join(self.img_dir, "right")
        self.scale = scale
        self.start_time = start_time
        self.end_time = end_time
        self.display = display

        os.makedirs(image_dir, exist_ok=True)

        suffix = "/compressed" if compressed else ""
        if self.stereo:
            self.img_topics = [left_topic + suffix, right_topic + suffix]
            os.makedirs(self.left_image_dir, exist_ok=True)
            os.makedirs(self.right_image_dir, exist_ok=True)
            self.synchronizer = ExactTimeSynchronizer(2, cache_size, self.write_stereo_images)
        else:
            self.img_topics = [left_topic + suffix]

        self.cv_bridge = CvBridge()

    def save_image(self, img_msg, base_dir: str, stamp_nsec: int):
        filename = os.path.join(base_dir, str(stamp_nsec) + ".png")
        if is_compressed_image(img_msg):
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg)
        else:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        h, w, c = cv_image.shape
        if self.scale != 1.0:
            cv_image = cv2.resize(cv_image, (int(w * self.scale), int(h * self.scale)))

        grayscale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        result = cv2.mean(hsv)
        if result[2] < 60:
            print("dark image. applying CLAHE")
            grayscale = clahe.apply(grayscale)

        cv2.imwrite(filename, grayscale)
        if self.display:
            cv2.imshow("image", grayscale)
            cv2.waitKey(1)

    def extract_images(self, delay: float = 0.0):
        reader = BagReader(self.bag, topics=self.img_topics)
        # Rate limit per topic, so that the left and right images of a stereo pair both pass
        start = reader.start_time_nsec() * 1e-9
        previous_stamps = {topic: start for topic in self.img_topics}

        print("topics: {}".format(self.img_topics))
        for topic, msg, t in tqdm(reader, total=reader.message_count()):
            if not self.node.ok():
                break
            t_sec = t * 1e-9
            if t_sec < self.start_time:
                continue
            if self.end_time > 0 and t_sec > self.end_time:
                break
            if t_sec - previous_stamps[topic] < delay:
                continue
            if self.stereo:
                self.synchronizer.add(self.img_topics.index(topic), msg)
            else:
                self.save_image(msg, self.img_dir, stamp_to_nsec(msg.header.stamp))
            previous_stamps[topic] = t_sec
        reader.close()

    def write_stereo_images(self, left_msg, right_msg):
        # Matches timestamps in left and right messages
        stamp = min(stamp_to_nsec(left_msg.header.stamp), stamp_to_nsec(right_msg.header.stamp))
        self.save_image(left_msg, self.left_image_dir, stamp)
        self.save_image(right_msg, self.right_image_dir, stamp)


if __name__ == "__main__":
    node = Node("extract_images")

    image_folder = node.get_param("image_dir", "")
    bag_file = node.get_param("bag", "")
    if not image_folder:
        node.logfatal("Require the dataset path of asl directory")
        exit(1)
    if not bag_file:
        node.logfatal("Require the bag file to extract images")
        exit(1)

    scale = node.get_param("scale", 1.0)
    delay = node.get_param("write_every_nsecs", 0.0)  # seconds between written images
    stereo = node.get_param("stereo", False)
    start_time = node.get_param("start_time", 0.0)  # bag time [s], 0: from the start
    end_time = node.get_param("end_time", 0.0)  # bag time [s], 0: until the end
    cache_size = node.get_param("cache_size", 100)
    left = node.get_param("left", "/left/image_raw")
    right = node.get_param("right", "/right/image_raw")
    compressed = node.get_param("compressed", False)
    display = node.get_param("display", False)

    node.loginfo("bag: {}".format(bag_file))
    node.loginfo("image dir: {}".format(image_folder))
    node.loginfo("Delay : {}".format(delay))
    node.loginfo("stereo: {}".format(stereo))
    node.loginfo("compressed: {}".format(compressed))
    node.loginfo("left image topic: {}".format(left))
    node.loginfo("right image topic: {}".format(right))

    extractor = ImageExtractor(
        node,
        image_dir=image_folder,
        bag_file=bag_file,
        stereo=stereo,
        left_topic=left,
        compressed=compressed,
        cache_size=cache_size,
        scale=scale,
        right_topic=right,
        start_time=start_time,
        end_time=end_time,
        display=display,
    )
    try:
        extractor.extract_images(delay=delay)
    except KeyboardInterrupt:
        print("Interrupted")
    node.shutdown()
