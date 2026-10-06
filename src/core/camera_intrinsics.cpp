#include "core/camera_intrinsics.hpp"

#include "utils/print.hpp"

namespace utils_ros {

void readIntrinsics(const cv::FileNode& cam_node, CameraIntrinsics& intrinsics) {
  cv::FileNode knode = cam_node["K"]["data"];
  if (knode.isSeq()) {
    intrinsics.K.at<double>(0, 0) = static_cast<double>(knode[0]);
    intrinsics.K.at<double>(1, 1) = static_cast<double>(knode[4]);
    intrinsics.K.at<double>(0, 2) = static_cast<double>(knode[2]);
    intrinsics.K.at<double>(1, 2) = static_cast<double>(knode[5]);
  } else {
    PRINT_ERROR("Camera Matrix is not a sequence");
  }

  cv::FileNode dnode = cam_node["D"]["data"];
  if (dnode.isSeq()) {
    intrinsics.D.at<double>(0, 0) = static_cast<double>(dnode[0]);
    intrinsics.D.at<double>(1, 0) = static_cast<double>(dnode[1]);
    intrinsics.D.at<double>(2, 0) = static_cast<double>(dnode[2]);
    intrinsics.D.at<double>(3, 0) = static_cast<double>(dnode[3]);
  } else {
    PRINT_ERROR("Distortion coeffs is not a sequence");
  }
}

}  // namespace utils_ros
