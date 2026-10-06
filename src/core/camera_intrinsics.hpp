#pragma once

#include <opencv2/core.hpp>

namespace utils_ros {

/// Pinhole camera matrix and 4 distortion coefficients (OpenCV radial-tangential model).
struct CameraIntrinsics {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
};

/// Reads "K" and "D" (OpenCV matrices) from a camera calibration node, e.g. config/gopro/*.yaml.
void readIntrinsics(const cv::FileNode& cam_node, CameraIntrinsics& intrinsics);

}  // namespace utils_ros
