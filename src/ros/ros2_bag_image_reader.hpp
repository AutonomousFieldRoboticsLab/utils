#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace utils_ros {

/**
 * @brief Reads the images of one topic from a ROS 2 bag (MCAP or SQLite3).
 *
 * Has the same interface as ROS1BagImageReader so the executables only differ in which one they
 * use.
 */
class ROS2BagImageReader {
public:
  /// @param compressed read sensor_msgs/CompressedImage instead of sensor_msgs/Image
  ROS2BagImageReader(const std::string& bag_path, const std::string& topic, bool compressed);

  /// Reads the next image (as BGR8) and its timestamp; returns false at the end of the bag.
  bool next(cv::Mat& image, std::uint64_t& stamp_ns);

private:
  rosbag2_cpp::Reader reader_;
  std::string topic_;
  bool compressed_;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_serialization_;
};

}  // namespace utils_ros
