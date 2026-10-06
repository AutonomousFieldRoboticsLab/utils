#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace utils_ros {

/// Reads the image timestamps [ns] from <folder_path>/data.csv (EuRoC/ASL format).
bool getImageStamps(const std::string& folder_path, std::vector<std::uint64_t>& time_stamps);

/// Reads the timestamps [s] in the first column of a trajectory file, converted to ns.
bool getStampsFromTrajectory(const std::string& trajectory_file,
                             std::vector<std::uint64_t>& time_stamps,
                             bool skip_first_line = false);

}  // namespace utils_ros
