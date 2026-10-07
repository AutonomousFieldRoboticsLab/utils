#include "core/trajectory.hpp"

#include <cassert>
#include <fstream>
#include <vector>

#include "utils/print.hpp"

namespace utils_ros {

Trajectory::Trajectory(const std::string& traj_file) : traj_file_(traj_file) {}

bool Trajectory::loadTrajectory(char separator, bool skip_first_line, bool double_stamp) {
  return loadTrajectory(traj_file_, separator, skip_first_line, double_stamp);
}

bool Trajectory::loadTrajectory(const std::string& traj_file,
                                char separator,
                                bool skip_first_line,
                                bool double_stamp) {
  stamp_to_index_map_.clear();
  poses_.clear();
  timestamps_.clear();

  PRINT_INFO("Parsing VIO trajectory data ....");
  std::ifstream fin(traj_file.c_str());

  if (!fin.is_open()) {
    PRINT_ERROR("Cannot open file: " << traj_file);
    return false;
  }

  // Skip the first line, containing the header.
  std::string line;
  if (skip_first_line) std::getline(fin, line);

  std::size_t index = 0;
  while (std::getline(fin, line)) {
    std::uint64_t timestamp = 0;
    std::vector<double> data_raw;
    for (std::size_t i = 0u; i < 9; i++) {
      std::size_t idx = line.find_first_of(separator);
      if (i == 0u) {
        if (double_stamp)
          timestamp = static_cast<std::uint64_t>(std::stold(line.substr(0, idx)) * 1000000000);
        else
          timestamp = std::stoll(line.substr(0, idx));
      } else {
        data_raw.push_back(std::stod(line.substr(0, idx)));
      }
      line = line.substr(idx + 1);
    }

    Pose pose;
    pose.position = Eigen::Vector3d(data_raw[0], data_raw[1], data_raw[2]);
    // Quaternion stored as x y z w
    pose.orientation = Eigen::Quaterniond(data_raw[6], data_raw[3], data_raw[4], data_raw[5]);

    stamp_to_index_map_.insert({timestamp, index});
    poses_.push_back(pose);
    timestamps_.insert(timestamp);
    index++;
  }

  PRINT_INFO("Added " << poses_.size() << " poses to trajectory.");
  assert(timestamps_.size() == poses_.size());
  assert(timestamps_.size() == stamp_to_index_map_.size());

  fin.close();
  return true;
}

bool Trajectory::getPose(std::uint64_t timestamp, Pose& pose) const {
  auto it = stamp_to_index_map_.find(timestamp);
  if (it == stamp_to_index_map_.end()) return false;

  pose = poses_[it->second];
  return true;
}

}  // namespace utils_ros
