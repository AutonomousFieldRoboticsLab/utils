#pragma once

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils_ros {

struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/// A VIO trajectory read from a text file with lines "timestamp x y z qx qy qz qw".
class Trajectory {
public:
  explicit Trajectory(const std::string& traj_file);

  bool loadTrajectory(char separator = ' ', bool skip_first_line = false, bool double_stamp = true);
  bool loadTrajectory(const std::string& traj_file,
                      char separator = ' ',
                      bool skip_first_line = false,
                      bool double_stamp = true);

  bool getPose(std::uint64_t timestamp, Pose& pose) const;

  const std::set<std::uint64_t>& getTimestamps() const { return timestamps_; }
  const std::vector<Pose>& getPoses() const { return poses_; }

private:
  std::string traj_file_;
  std::unordered_map<std::uint64_t, std::size_t> stamp_to_index_map_;
  std::set<std::uint64_t> timestamps_;
  std::vector<Pose> poses_;
};

}  // namespace utils_ros
