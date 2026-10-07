#include "ros/ros1_bag_image_reader.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace utils_ros {

ROS1BagImageReader::ROS1BagImageReader(const std::string& bag_path,
                                       const std::string& topic,
                                       bool compressed)
    : compressed_(compressed) {
  bag_.open(bag_path, rosbag::bagmode::Read);
  view_ = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery(topic));
  it_ = view_->begin();
}

bool ROS1BagImageReader::next(cv::Mat& image, std::uint64_t& stamp_ns) {
  while (it_ != view_->end()) {
    // Copy: the iterator frees the instance it refers to when it is advanced
    const rosbag::MessageInstance message = *it_;
    ++it_;

    if (compressed_) {
      auto image_msg = message.instantiate<sensor_msgs::CompressedImage>();
      if (!image_msg) continue;
      image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      stamp_ns = image_msg->header.stamp.toNSec();
    } else {
      auto image_msg = message.instantiate<sensor_msgs::Image>();
      if (!image_msg) continue;
      image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      stamp_ns = image_msg->header.stamp.toNSec();
    }
    return true;
  }
  return false;
}

}  // namespace utils_ros
