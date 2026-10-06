#include "ros/ros2_bag_image_reader.hpp"

#include <memory>

#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>

// cv_bridge ships cv_bridge.h up to Humble and cv_bridge.hpp from Jazzy on.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace utils_ros {

namespace {

std::uint64_t toNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

}  // namespace

ROS2BagImageReader::ROS2BagImageReader(const std::string& bag_path,
                                       const std::string& topic,
                                       bool compressed)
    : topic_(topic), compressed_(compressed) {
  reader_.open(bag_path);

  // Only read the messages of the requested topic
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic_);
  reader_.set_filter(filter);
}

bool ROS2BagImageReader::next(cv::Mat& image, std::uint64_t& stamp_ns) {
  while (reader_.has_next()) {
    auto bag_message = reader_.read_next();
    if (bag_message->topic_name != topic_) continue;

    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
    if (compressed_) {
      auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      compressed_serialization_.deserialize_message(&serialized_msg, image_msg.get());
      image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      stamp_ns = toNanoseconds(image_msg->header.stamp);
    } else {
      auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
      image_serialization_.deserialize_message(&serialized_msg, image_msg.get());
      image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      stamp_ns = toNanoseconds(image_msg->header.stamp);
    }
    return true;
  }
  return false;
}

}  // namespace utils_ros
