#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace utils_ros {

/**
 * @brief Reads the images of one topic from a ROS 1 bag.
 *
 * Has the same interface as ROS2BagImageReader so the executables only differ in which one they
 * use.
 */
class ROS1BagImageReader {
public:
  /// @param compressed read sensor_msgs/CompressedImage instead of sensor_msgs/Image
  ROS1BagImageReader(const std::string& bag_path, const std::string& topic, bool compressed);

  /// Reads the next image (as BGR8) and its timestamp; returns false at the end of the bag.
  bool next(cv::Mat& image, std::uint64_t& stamp_ns);

private:
  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::iterator it_;
  bool compressed_;
};

}  // namespace utils_ros
