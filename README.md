# utils_ros

Utility nodes and scripts used in several SLAM projects. The main tool writes the **keyframe images**
of a VIO trajectory from a bag, **undistorted** with the camera calibration, e.g. as input for COLMAP.

## Supported platforms

| ROS | Distro | Ubuntu | Docker image |
|---|---|---|---|
| ROS 1 | Noetic | 20.04 | `docker/Dockerfile_ros1_20_04` |
| ROS 2 | Humble | 22.04 | `docker/Dockerfile_ros2_22_04` |
| ROS 2 | Jazzy | 24.04 | `docker/Dockerfile_ros2_24_04` |

The same package builds for both ROS versions: CMake detects whether catkin or ament is sourced and
builds the matching executables.

## Tools

| Tool | ROS | Description |
|---|---|---|
| `write_kf_images` | ROS 1 and ROS 2 | Writes the keyframe images of a VIO trajectory from a bag, undistorted and optionally scaled |
| `scripts/extract_images.py` | ROS 1 and ROS 2 | Writes images from a bag at a fixed rate (mono or stereo) |
| `scripts/extract_bag_stereo.py` | ROS 1 and ROS 2 | Writes synchronized stereo images from a bag, optionally undistorted |
| `scripts/gopro_combine_bags.py` | ROS 1 and ROS 2 | Combines a left and a right GoPro bag into one stereo bag |
| `scripts/brisk_feature_detection.py` | none | Detects and draws BRISK features in an image or a folder of images |

# Installation

## Docker (recommended)

[`docker-compose.yml`](docker-compose.yml) defines one service per distro: `utils_ros_noetic`,
`utils_ros_humble` and `utils_ros_jazzy`. The folder in `DATA_DIR` is mounted at `/utils_ws/data`
(default: `./data`); you can set it once in a `.env` file next to `docker-compose.yml`:

```bash
echo "DATA_DIR=/path/to/your/data" > .env
```

Build an image from the repository root:

```bash
docker compose build utils_ros_jazzy
```

Run `docker compose run --rm utils_ros_jazzy` for an interactive shell, or pass a command directly (see
[Usage](#usage)). Containers run as root by default, so output files are owned by root. To keep your
own user, add `--user $(id -u):$(id -g) -e HOME=/tmp` to `docker compose run`.

## Build from source

All dependencies (ROS packages, OpenCV, Eigen) are declared in `package.xml` and installed by
`rosdep`.

### ROS 2 (Humble / Jazzy)

```bash
mkdir -p ~/utils_ws/src && cd ~/utils_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/utils_ros2.git
cd ~/utils_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select utils_ros
source install/setup.bash
```

### ROS 1 (Noetic)

```bash
mkdir -p ~/utils_ws/src && cd ~/utils_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/utils_ros2.git
cd ~/utils_ws
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```

# Usage

## Write keyframe images

`write_kf_images` reads the keyframe timestamps from a VIO trajectory, finds the images with exactly
these timestamps in the bag, undistorts them with the camera calibration and writes them as
`<image_dir>/<timestamp_ns>.png`.

ROS 2:

```bash
ros2 launch utils_ros write_keyframe_images.launch.py \
    bag_file:=/path/to/bag \
    traj_file:=/path/to/trajectory.txt \
    image_dir:=/path/to/output/images \
    config_file:=/path/to/camera.yaml
```

ROS 1:

```bash
roslaunch utils_ros write_keyframe_images.launch \
    bag_file:=/path/to/bag.bag \
    traj_file:=/path/to/trajectory.txt \
    image_dir:=/path/to/output/images \
    config_file:=/path/to/camera.yaml
```

With Docker, the same command runs in a container, with paths under `/utils_ws/data`:

```bash
docker compose run --rm utils_ros_jazzy ros2 launch utils_ros write_keyframe_images.launch.py bag_file:=/utils_ws/data/run traj_file:=/utils_ws/data/trajectory.txt image_dir:=/utils_ws/data/keyframes
```

| Parameter | Launch default | Description |
|---|---|---|
| `bag_file` | | Input bag (ROS 2: MCAP or SQLite3 bag directory; ROS 1: `.bag` file) |
| `traj_file` | | VIO trajectory, see the format below |
| `image_dir` | | Output directory (`left/` and `right/` subfolders for stereo) |
| `config_file` | `config/gopro/gopro1.yaml` | Camera calibration, see below |
| `left_image_topic` | `/gopro/image_raw` | Image topic (left camera for stereo) |
| `right_image_topic` | | Right image topic, stereo only |
| `compressed` | `true` | Read `sensor_msgs/CompressedImage` from `<topic>/compressed` |
| `scale` | `1.0` | Scaling factor of the written images, applied after undistortion |
| `skip_first_line` | `true` | Skip the header line of the trajectory file |
| `stereo` | `false` | Write left and right images |

### Trajectory format

One keyframe per line, separated by spaces, with the timestamp in seconds:

```
timestamp tx ty tz qx qy qz qw
```

A keyframe is written only if its timestamp matches an image timestamp in the bag exactly (to the
nanosecond).

### Camera calibration

The calibration files in [`config/gopro`](config/gopro) are OpenCV YAML files with the image size,
the camera matrix `K` and four distortion coefficients `D` (OpenCV radial-tangential model:
k1, k2, p1, p2):

| File | Resolution |
|---|---|
| `gopro1.yaml` | 960x540 |
| `gopro2.yaml` | 960x540 |
| `gopro10.yaml` | 1920x1080 |

A calibration only fits one camera at one resolution. For bags written by
[gopro_ros](https://github.com/AutonomousFieldRoboticsLab/gopro_ros2) with `scale:=0.5`, use a
960x540 calibration; for full-resolution bags use a 1920x1080 one. For stereo (`stereo:=true`), the
file needs a `left` and a `right` section, each with its own `K` and `D`.

## Python tools

The scripts in [`scripts/`](scripts) run on ROS 1 and ROS 2 and are installed with both builds. They
share [`scripts/ros_compat.py`](scripts/ros_compat.py), which picks the ROS 1 (`rospy`, `rosbag`) or
ROS 2 (`rclpy`, `rosbag2_py`) API from `$ROS_VERSION`. On ROS 2 they read MCAP and SQLite3 bags.

`extract_images.py` and `extract_bag_stereo.py` have launch files with the same arguments for both
ROS versions:

```bash
ros2 launch utils_ros extract_images.launch.py bag:=/path/to/bag image_dir:=/path/to/images
```

```bash
ros2 launch utils_ros extract_bag_stereo.launch.py bag:=/path/to/bag image_dir:=/path/to/images config_file:=/path/to/stereo.yaml
```

On ROS 1, use `roslaunch utils_ros extract_images.launch` and `roslaunch utils_ros
extract_bag_stereo.launch`. For stereo, `extract_images` pairs left and right images with identical
timestamps; `start_time`/`end_time` limit the extracted bag time range.

`gopro_combine_bags.py` takes command-line arguments. On ROS 2, `--storage_id` selects the storage
of the output bag (`.mcap`, the default, or `.db3`):

```bash
ros2 run utils_ros gopro_combine_bags.py -l /path/to/left -r /path/to/right -o /path/to/stereo -s .mcap
```

```bash
rosrun utils_ros gopro_combine_bags.py -l left.bag -r right.bag -o stereo.bag
```

# Notes

- ROS 2 Humble cannot read bags written by ROS 2 Jazzy (newer bag metadata format). Read Jazzy bags
  with the Jazzy image or build.

# Repository layout

```
src/
  core/        Trajectory, camera calibration and keyframe writer; ROS-agnostic
  utils/       Timestamp helpers and logging; ROS-agnostic
  ros/         ROS 1 and ROS 2 bag image readers with the same interface
  write_keyframe_images.cpp
cmake/         ROS1.cmake and ROS2.cmake
config/        Camera calibrations
launch/        ROS 1 (.launch) and ROS 2 (.launch.py) launch files
scripts/       Python tools (ROS 1 and ROS 2)
docker/        Dockerfiles for Noetic, Humble and Jazzy
docker-compose.yml   One service per distro
```

Code is formatted with the repository's `.clang-format`:

```bash
clang-format -i src/core/*.?pp src/utils/*.?pp src/ros/*.?pp src/*.cpp
```

# License

BSD 3-Clause, see [LICENSE](LICENSE).
