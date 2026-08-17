# AFRL Utils

## Python Scripts

This folder contains python scripts that are used for various tasks that do not need to run as ros nodes.
The README.md file inside the folder contains more information on the scripts in that folder.

The scripts that import `rospy` or `rosbag` are legacy ROS 1 utilities and are
not installed as part of this ROS 2 package. Run those scripts in a compatible
ROS 1 environment. Their additional Python dependencies are OpenCV, NumPy, and
tqdm (`python3-opencv`, `python3-numpy`, and `python3-tqdm` on Ubuntu).

## ROS2 Package

### Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy (verified)
- Eigen3
- OpenCV

Install ROS 2 first by following the
[official installation guide](https://docs.ros.org/en/jazzy/Installation.html),
then install the package dependencies:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update

# ROS 2 dependencies
sudo apt-get install -y \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-rosbag2-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
    ros-$ROS_DISTRO-ros2launch

# System development libraries
sudo apt-get install -y \
    libeigen3-dev \
    libopencv-dev
```

### Build

```bash
mkdir -p ~/utils_ros2_ws/src 
cd ~/utils_ros2_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/utils_ros2.git
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --packages-select utils_ros2 --symlink-install
source ~/utils_ros2_ws/install/setup.bash # Or add this to ~/.bashrc to make it permanent
```

### Usage 

#### Extract Keyframes
To extract keyframes from a ros2bag, run the following command:
```bash
ros2 launch utils_ros2 write_keyframe_images.xml bag_file:=bag_file image_dir:=dir_to_save_images traj_file:=VIO_trajectory_file config_file:=camera config file
```

Note: By default, the keyframes are undistorted. The reason being COLMAP undistortion changes image size in an effort to maximize information from images.

The gopro config files are inside the ```config/gopro``` folder. 

Check the launch file for additional parameters.
