#include "utils/stamp_utils.hpp"

#include <fstream>

#include "utils/print.hpp"

namespace utils_ros {

bool getImageStamps(const std::string& folder_path, std::vector<std::uint64_t>& time_stamps) {
  if (folder_path.empty()) {
    PRINT_ERROR("Camera Folder Empty: " << folder_path);
    return false;
  }
  const std::string stamp_file = folder_path + "/" + "data.csv";

  std::ifstream fin(stamp_file.c_str());
  if (!fin.is_open()) {
    PRINT_ERROR("Cannot open file: " << stamp_file);
    return false;
  }

  // Skip the first line, containing the header.
  std::string item;
  std::getline(fin, item);

  // Read/store list of image names.
  while (std::getline(fin, item)) {
    auto idx = item.find_first_of(',');
    std::uint64_t timestamp = std::stoll(item.substr(0, idx));
    time_stamps.push_back(timestamp);
  }

  fin.close();
  return true;
}

bool getStampsFromTrajectory(const std::string& trajectory_file,
                             std::vector<std::uint64_t>& time_stamps,
                             bool skip_first_line) {
  std::ifstream fin(trajectory_file.c_str());
  if (!fin.is_open()) {
    PRINT_ERROR("Cannot open file: " << trajectory_file);
    return false;
  }

  // Skip the first line, containing the header.
  std::string item;
  if (skip_first_line) std::getline(fin, item);

  while (std::getline(fin, item)) {
    auto idx = item.find_first_of(' ');
    std::uint64_t timestamp =
        static_cast<std::uint64_t>(std::stold(item.substr(0, idx)) * 1000000000);
    time_stamps.push_back(timestamp);
  }

  fin.close();
  return true;
}

}  // namespace utils_ros
