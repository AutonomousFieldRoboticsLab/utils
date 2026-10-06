## Python Scripts

Tools for ROS 1 and ROS 2, installed with both builds (`rosrun utils_ros <script>` /
`ros2 run utils_ros <script>`). On ROS 2 they read MCAP and SQLite3 bags.

- `extract_images.py` - Writes images from a bag at a fixed rate, mono or stereo (`launch/extract_images.launch[.py]`)
- `extract_bag_stereo.py` - Writes synchronized stereo images from a bag, optionally undistorted (`launch/extract_bag_stereo.launch[.py]`)
- `gopro_combine_bags.py` - Combines a left and a right GoPro bag into one stereo bag (`--storage_id .mcap|.db3` on ROS 2)
- `brisk_feature_detection.py` - Detects and draws BRISK features in an image or a folder (no ROS needed)
- `ros_compat.py` - Shared module: ROS 1 / ROS 2 parameters, logging, bag reading and writing

The scripts without a launch file take command-line arguments:

```
python3 script_name.py --help
```
