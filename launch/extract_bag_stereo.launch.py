"""ROS 2 launch file for scripts/extract_bag_stereo.py. For ROS 1 use extract_bag_stereo.launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (launch argument, node parameter, default, description)
STRING_ARGS = [
    ("image_dir", "image_dir", "/path/to/images", "Output directory"),
    ("bag", "input_bag", "/path/to/bag", "Input bag (MCAP or SQLite3)"),
    ("left_topic", "left_topic", "/slave1/image_raw", "Left image topic"),
    ("right_topic", "right_topic", "/slave2/image_raw", "Right image topic"),
    (
        "config_file",
        "config_file",
        "/path/to/stereo_config.yaml",
        'Stereo calibration with "left" and "right" sections',
    ),
]
VALUE_ARGS = [
    ("scale", "scale", "1.0", "Scaling factor of the written images"),
    ("freq", "freq", "2.0", "Written stereo pairs per second"),
    ("undistort", "undistort", "false", "Undistort the images with the calibration"),
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
        executable="extract_bag_stereo.py",
        name="bag_extract_stereo",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription(args + [node])
