#include "core/keyframe_writer.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/print.hpp"

namespace utils_ros {

KeyframeWriter::KeyframeWriter(const CameraIntrinsics& intrinsics,
                               const std::set<std::uint64_t>& keyframe_stamps,
                               const std::string& output_dir,
                               double scale)
    : intrinsics_(intrinsics),
      keyframe_stamps_(keyframe_stamps),
      output_dir_(output_dir),
      scale_(scale) {}

void KeyframeWriter::initialize(const cv::Size& image_size) {
  output_size_ = cv::Size(static_cast<int>(image_size.width * scale_),
                          static_cast<int>(image_size.height * scale_));

  PRINT_INFO("Original height, width: " << image_size.height << ", " << image_size.width);
  PRINT_INFO("New height, width: " << output_size_.height << ", " << output_size_.width);

  cv::initUndistortRectifyMap(
      intrinsics_.K, intrinsics_.D, cv::Mat(), intrinsics_.K, image_size, CV_32FC1, map_x_, map_y_);
  initialized_ = true;
}

bool KeyframeWriter::process(const cv::Mat& image, std::uint64_t stamp_ns) {
  if (!initialized_) initialize(image.size());

  if (keyframe_stamps_.find(stamp_ns) == keyframe_stamps_.end()) return false;

  cv::remap(image, undistorted_image_, map_x_, map_y_, cv::INTER_LINEAR);
  if (scale_ != 1.0) cv::resize(undistorted_image_, undistorted_image_, output_size_);
  cv::imwrite(output_dir_ + "/" + std::to_string(stamp_ns) + ".png", undistorted_image_);
  return true;
}

}  // namespace utils_ros
