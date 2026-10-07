"""ROS 2 launch file for scripts/extract_images.py. For ROS 1 use extract_images.launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (launch argument, node parameter, default, description)
STRING_ARGS = [
    ("image_dir", "image_dir", "/path/to/images", "Output directory"),
    ("bag", "bag", "/path/to/bag", "Input bag (MCAP or SQLite3)"),
    ("left", "left", "/left/image_raw", "Image topic (left camera for stereo)"),
    ("right", "right", "", "Right image topic, stereo only"),
]
VALUE_ARGS = [
    ("scale", "scale", "0.5", "Scaling factor of the written images"),
    ("write_every_secs", "write_every_nsecs", "0.25", "Seconds between written images"),
    ("compressed", "compressed", "true", "Read sensor_msgs/CompressedImage (<topic>/compressed)"),
    ("stereo", "stereo", "false", "Write synchronized left and right images"),
    ("start_time", "start_time", "0.0", "Bag time [s] to start from, 0: from the start"),
    ("end_time", "end_time", "0.0", "Bag time [s] to stop at, 0: until the end"),
    ("display", "display", "false", "Show the images while writing"),
]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(arg, default_value=default, description=description)
        for arg, _, default, description in STRING_ARGS + VALUE_ARGS
    ]
    parameters = {
        param: ParameterValue(LaunchConfiguration(arg), value_type=str)
        for arg, param, _, _ in STRING_ARGS
    }
    parameters.update({param: LaunchConfiguration(arg) for arg, param, _, _ in VALUE_ARGS})

    node = Node(
        package="utils_ros",
        executable="extract_images.py",
        name="extract_images",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription(args + [node])
