"""ROS 1 / ROS 2 compatibility layer for the Python tools in this folder.

The ROS version is taken from $ROS_VERSION (set by the ROS setup scripts). The tools only use the
functions and classes below, so they run unchanged on ROS 1 (rospy, rosbag) and ROS 2 (rclpy,
rosbag2_py).
"""

import os
from collections import OrderedDict

ROS_VERSION = int(os.environ.get("ROS_VERSION", "1"))

if ROS_VERSION == 1:
    import rosbag
    import rospy
else:
    import rclpy
    import rosbag2_py
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message


def stamp_to_nsec(stamp) -> int:
    """Converts a header stamp (ROS 1 rospy.Time or ROS 2 builtin_interfaces/Time) to ns."""
    if ROS_VERSION == 1:
        return stamp.to_nsec()
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def message_type(msg) -> str:
    """Returns the message type name, e.g. "CompressedImage".

    Messages read from a ROS 1 bag are instances of generated classes, so their Python class name
    differs from the message type.
    """
    if ROS_VERSION == 1:
        return msg._type.split("/")[-1]
    return type(msg).__name__


def is_compressed_image(msg) -> bool:
    return message_type(msg) == "CompressedImage"


class Node:
    """A ROS node providing private parameters and logging."""

    def __init__(self, name: str):
        if ROS_VERSION == 1:
            rospy.init_node(name, anonymous=True)
        else:
            rclpy.init()
            self._node = rclpy.create_node(name)

    def get_param(self, name: str, default):
        """Returns the private parameter `name`, or `default` if it is not set."""
        if ROS_VERSION == 1:
            return rospy.get_param("~" + name, default)
        descriptor = ParameterDescriptor(dynamic_typing=True)
        return self._node.declare_parameter(name, default, descriptor).value

    def loginfo(self, message: str):
        if ROS_VERSION == 1:
            rospy.loginfo(message)
        else:
            self._node.get_logger().info(message)

    def logfatal(self, message: str):
        if ROS_VERSION == 1:
            rospy.logfatal(message)
        else:
            self._node.get_logger().fatal(message)

    def ok(self) -> bool:
        if ROS_VERSION == 1:
            return not rospy.is_shutdown()
        return rclpy.ok()

    def shutdown(self):
        if ROS_VERSION == 1:
            rospy.signal_shutdown("done")
        elif rclpy.ok():
            self._node.destroy_node()
            rclpy.shutdown()


def _ros2_storage_options(uri: str) -> "rosbag2_py.StorageOptions":
    metadata = rosbag2_py.Info().read_metadata(uri, "")
    return rosbag2_py.StorageOptions(uri=uri, storage_id=metadata.storage_identifier)


class BagReader:
    """Reads messages from a ROS 1 bag file or a ROS 2 bag directory (MCAP or SQLite3)."""

    def __init__(self, path: str, topics=None, deserialize: bool = True):
        """
        :param topics: only read these topics (all topics if None)
        :param deserialize: ROS 2 only; if False, messages are returned as serialized bytes
        """
        self.path = path
        self.topics = list(topics) if topics else None
        self.deserialize = deserialize

        if ROS_VERSION == 1:
            self._bag = rosbag.Bag(path, "r")
        else:
            storage_options = _ros2_storage_options(path)
            self._metadata = rosbag2_py.Info().read_metadata(path, storage_options.storage_id)
            self._reader = rosbag2_py.SequentialReader()
            self._reader.open(storage_options, rosbag2_py.ConverterOptions("cdr", "cdr"))
            if self.topics:
                self._reader.set_filter(rosbag2_py.StorageFilter(topics=self.topics))
            self._types = {t.name: t.type for t in self._reader.get_all_topics_and_types()}

    def topic_types(self) -> dict:
        """Returns {topic: message type name}."""
        if ROS_VERSION == 1:
            return {t: v.msg_type for t, v in self._bag.get_type_and_topic_info().topics.items()}
        return dict(self._types)

    def message_count(self) -> int:
        if ROS_VERSION == 1:
            return self._bag.get_message_count(self.topics)
        return sum(
            t.message_count
            for t in self._metadata.topics_with_message_count
            if not self.topics or t.topic_metadata.name in self.topics
        )

    def start_time_nsec(self) -> int:
        """Receive time of the first message in the bag (all topics), in ns."""
        if ROS_VERSION == 1:
            return int(round(self._bag.get_start_time() * 1e9))
        return self._metadata.starting_time.nanoseconds

    def __iter__(self):
        """Yields (topic, message, bag receive time in ns)."""
        if ROS_VERSION == 1:
            for topic, msg, t in self._bag.read_messages(topics=self.topics):
                yield topic, msg, t.to_nsec()
        else:
            message_classes = {}
            while self._reader.has_next():
                topic, data, t = self._reader.read_next()
                if not self.deserialize:
                    yield topic, data, t
                    continue
                if topic not in message_classes:
                    message_classes[topic] = get_message(self._types[topic])
                yield topic, deserialize_message(data, message_classes[topic]), t

    def close(self):
        if ROS_VERSION == 1:
            self._bag.close()


# ROS 2 storage selectors, as used by gopro_ros: ".mcap" or ".db3"
ROS2_STORAGE_IDS = {".mcap": "mcap", ".db3": "sqlite3"}


class BagWriter:
    """Writes messages to a ROS 1 bag file or a ROS 2 bag directory (MCAP or SQLite3)."""

    def __init__(self, path: str, storage_id: str = ".mcap"):
        """:param storage_id: ROS 2 only, ".mcap" or ".db3" """
        if ROS_VERSION == 1:
            self._bag = rosbag.Bag(path, "w")
        else:
            if storage_id not in ROS2_STORAGE_IDS:
                raise ValueError(f"storage_id must be one of {list(ROS2_STORAGE_IDS)}")
            self._writer = rosbag2_py.SequentialWriter()
            self._writer.open(
                rosbag2_py.StorageOptions(uri=path, storage_id=ROS2_STORAGE_IDS[storage_id]),
                rosbag2_py.ConverterOptions("cdr", "cdr"),
            )
            self._topics = set()

    def write(self, topic: str, msg, t_nsec: int, msg_type: str = None):
        """Writes a message; on ROS 2, `msg` may be serialized bytes and `msg_type` is required."""
        if ROS_VERSION == 1:
            self._bag.write(topic, msg, rospy.Time(nsecs=t_nsec))
            return

        if topic not in self._topics:
            try:  # Jazzy and newer need a topic id
                metadata = rosbag2_py.TopicMetadata(
                    id=len(self._topics), name=topic, type=msg_type, serialization_format="cdr"
                )
            except TypeError:  # Humble
                metadata = rosbag2_py.TopicMetadata(
                    name=topic, type=msg_type, serialization_format="cdr"
                )
            self._writer.create_topic(metadata)
            self._topics.add(topic)
        self._writer.write(topic, msg, t_nsec)

    def close(self):
        if ROS_VERSION == 1:
            self._bag.close()
        else:
            del self._writer


class ExactTimeSynchronizer:
    """Pairs messages of several inputs with identical header stamps.

    Same behavior as message_filters.TimeSynchronizer, whose Python API differs between ROS 1 and
    ROS 2: once every input has a message with the same stamp, the callback is called with them and
    all older messages are dropped.
    """

    def __init__(self, num_inputs: int, queue_size: int, callback):
        self._queues = [OrderedDict() for _ in range(num_inputs)]
        self._queue_size = queue_size
        self._callback = callback

    def add(self, index: int, msg):
        stamp = stamp_to_nsec(msg.header.stamp)
        queue = self._queues[index]
        queue[stamp] = msg
        while len(queue) > self._queue_size:
            queue.popitem(last=False)

        if all(stamp in q for q in self._queues):
            msgs = [q[stamp] for q in self._queues]
            for q in self._queues:
                for old in [s for s in q if s <= stamp]:
                    del q[old]
            self._callback(*msgs)
