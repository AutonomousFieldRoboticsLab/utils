"""ROS 2 launch file. For ROS 1 use write_keyframe_images.launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_CONFIG = os.path.join(
    get_package_share_directory("utils_ros"), "config", "gopro", "gopro1.yaml"
)

# (name, default, description)
STRING_ARGS = [
    ("bag_file", "/path/to/bag", "Input bag"),
    ("image_dir", "/path/to/images", "Output directory for the keyframe images"),
    ("traj_file", "/path/to/trajectory.txt", "VIO keyframe trajectory"),
    ("left_image_topic", "/gopro/image_raw", "Image topic (left camera for stereo)"),
    ("right_image_topic", "", "Right image topic (stereo only)"),
    ("config_file", DEFAULT_CONFIG, "Camera calibration (config/gopro/*.yaml)"),
]
VALUE_ARGS = [
    ("scale", "1.0", "Scaling factor of the written images"),
    ("compressed", "true", "Read sensor_msgs/CompressedImage (<topic>/compressed)"),
    ("skip_first_line", "true", "Skip the header line of the trajectory file"),
    ("stereo", "false", "Write left and right images"),
]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in STRING_ARGS + VALUE_ARGS
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name, _, _ in STRING_ARGS
    }
    parameters.update({name: LaunchConfiguration(name) for name, _, _ in VALUE_ARGS})

    node = Node(
        package="utils_ros",
        executable="write_kf_images",
        name="extract_keyframes",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription(args + [node])
