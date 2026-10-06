#pragma once

#include <cstdint>
#include <set>
#include <string>

#include <opencv2/core.hpp>

#include "core/camera_intrinsics.hpp"

namespace utils_ros {

/// Undistorts, scales and writes the images whose timestamps are keyframes.
class KeyframeWriter {
public:
  KeyframeWriter(const CameraIntrinsics& intrinsics,
                 const std::set<std::uint64_t>& keyframe_stamps,
                 const std::string& output_dir,
                 double scale);

  /// Writes <output_dir>/<stamp_ns>.png if stamp_ns is a keyframe; returns whether it was written.
  bool process(const cv::Mat& image, std::uint64_t stamp_ns);

private:
  /// Builds the undistortion maps for the size of the first image.
  void initialize(const cv::Size& image_size);

  CameraIntrinsics intrinsics_;
  const std::set<std::uint64_t>& keyframe_stamps_;
  std::string output_dir_;
  double scale_;

  bool initialized_ = false;
  cv::Size output_size_;
  cv::Mat map_x_, map_y_;
  cv::Mat undistorted_image_;
};

}  // namespace utils_ros
