# ===========================
# ROS 2 (ament) build
# ===========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(ament_libraries
  rclcpp
  sensor_msgs
  cv_bridge
  rosbag2_cpp
)

utils_add_core_library()

# ===========================
# Executables
# ===========================
add_executable(write_kf_images
  src/write_keyframe_images.cpp
  src/ros/ros2_bag_image_reader.cpp
)
ament_target_dependencies(write_kf_images ${ament_libraries})
target_compile_definitions(write_kf_images PRIVATE ROS_AVAILABLE=2)
target_link_libraries(write_kf_images ${PROJECT_NAME}_lib)
install(TARGETS write_kf_images
  DESTINATION lib/${PROJECT_NAME}
)

# Python tools and the ROS 1 / ROS 2 module they share
install(PROGRAMS
  scripts/brisk_feature_detection.py
  scripts/extract_bag_stereo.py
  scripts/extract_images.py
  scripts/gopro_combine_bags.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/ros_compat.py DESTINATION lib/${PROJECT_NAME})

# ===========================
# Install
# ===========================
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
