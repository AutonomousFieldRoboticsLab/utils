// Writes the keyframe images of a VIO trajectory from a bag, undistorted and optionally scaled,
// e.g. as input for COLMAP.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <set>
#include <string>

#include <opencv2/core.hpp>

#include "core/camera_intrinsics.hpp"
#include "core/keyframe_writer.hpp"
#include "core/trajectory.hpp"
#include "utils/print.hpp"

#if ROS_AVAILABLE == 1
#include <ros/ros.h>

#include "ros/ros1_bag_image_reader.hpp"
using BagImageReader = utils_ros::ROS1BagImageReader;
#elif ROS_AVAILABLE == 2
#include <rclcpp/rclcpp.hpp>

#include "ros/ros2_bag_image_reader.hpp"
using BagImageReader = utils_ros::ROS2BagImageReader;
#endif

namespace fs = std::filesystem;

namespace {

void shutdown() {
#if ROS_AVAILABLE == 1
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif
}

bool rosOk() {
#if ROS_AVAILABLE == 1
  return ros::ok();
#elif ROS_AVAILABLE == 2
  return rclcpp::ok();
#endif
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string traj_file;
  std::string bag_file;
  std::string left_image_topic;
  std::string right_image_topic;
  std::string image_dir;
  std::string config_file;
  bool compressed;
  bool skip_first_line;
  bool stereo;
  double scale;

#if ROS_AVAILABLE == 1
  ros::init(argc, argv, "write_images_from_list");
  ros::NodeHandle nh("~");
  nh.param<std::string>("traj_file", traj_file, "");
  nh.param<std::string>("bag_file", bag_file, "");
  nh.param<std::string>("left_image_topic", left_image_topic, "");
  nh.param<std::string>("right_image_topic", right_image_topic, "");
  nh.param<std::string>("image_dir", image_dir, "");
  nh.param<std::string>("config_file", config_file, "");
  nh.param<bool>("compressed", compressed, false);
  nh.param<bool>("skip_first_line", skip_first_line, false);
  nh.param<bool>("stereo", stereo, false);
  nh.param<double>("scale", scale, 1.0);
#elif ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("write_images_from_list");
  traj_file = node->declare_parameter<std::string>("traj_file", "");
  bag_file = node->declare_parameter<std::string>("bag_file", "");
  left_image_topic = node->declare_parameter<std::string>("left_image_topic", "");
  right_image_topic = node->declare_parameter<std::string>("right_image_topic", "");
  image_dir = node->declare_parameter<std::string>("image_dir", "");
  config_file = node->declare_parameter<std::string>("config_file", "");
  compressed = node->declare_parameter<bool>("compressed", false);
  skip_first_line = node->declare_parameter<bool>("skip_first_line", false);
  stereo = node->declare_parameter<bool>("stereo", false);
  scale = node->declare_parameter<double>("scale", 1.0);
#endif

  const std::pair<const std::string*, const char*> required[] = {
      {&traj_file, "VIO keyframe trajectory file"},
      {&bag_file, "Bag file"},
      {&left_image_topic, "Image topic"},
      {&image_dir, "Output directory"},
      {&config_file, "Config file"},
  };
  for (const auto& [value, name] : required) {
    if (value->empty()) {
      PRINT_ERROR(name << " not set from params");
      shutdown();
      return 1;
    }
  }
  if (stereo && right_image_topic.empty()) {
    PRINT_ERROR("right image topic not set from params");
    shutdown();
    return 1;
  }

  // Camera calibration: the whole file for a single camera, "left"/"right" sections for stereo
  cv::FileStorage fs_settings(config_file, cv::FileStorage::READ);
  if (!fs_settings.isOpened()) PRINT_ERROR("Wrong path to intrinsics: " << config_file);

  utils_ros::CameraIntrinsics left_intrinsics, right_intrinsics;
  if (stereo) {
    utils_ros::readIntrinsics(fs_settings["left"], left_intrinsics);
    utils_ros::readIntrinsics(fs_settings["right"], right_intrinsics);
  } else {
    utils_ros::readIntrinsics(fs_settings.root(), left_intrinsics);
  }

  utils_ros::Trajectory trajectory(traj_file);
  trajectory.loadTrajectory(' ', skip_first_line, true);
  const std::set<std::uint64_t>& kf_stamps = trajectory.getTimestamps();

  const std::string left_image_dir = stereo ? image_dir + "/left" : image_dir;
  const std::string right_image_dir = image_dir + "/right";
  fs::create_directories(left_image_dir);
  if (stereo) fs::create_directories(right_image_dir);

  if (compressed) {
    left_image_topic += "/compressed";
    right_image_topic += "/compressed";
  }

  cv::Mat image;
  std::uint64_t stamp_ns;

  // Left (or only) camera
  std::size_t kf_images = 0;
  std::size_t total_images = 0;
  {
    BagImageReader reader(bag_file, left_image_topic, compressed);
    utils_ros::KeyframeWriter writer(left_intrinsics, kf_stamps, left_image_dir, scale);
    while (rosOk() && reader.next(image, stamp_ns)) {
      total_images++;
      if (writer.process(image, stamp_ns)) kf_images++;
    }
  }

  // Right camera
  if (stereo) {
    BagImageReader reader(bag_file, right_image_topic, compressed);
    utils_ros::KeyframeWriter writer(right_intrinsics, kf_stamps, right_image_dir, scale);
    while (rosOk() && reader.next(image, stamp_ns)) writer.process(image, stamp_ns);
  }

  if (!rosOk()) {
    PRINT_WARNING("Interrupted");
    return 0;
  }

  PRINT_INFO("Wrote " << kf_images << "/" << total_images << " images");
  shutdown();
  return 0;
}
