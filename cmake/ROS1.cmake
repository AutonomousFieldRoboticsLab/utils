# ===========================
# ROS 1 (catkin) build
# ===========================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS roscpp rosbag sensor_msgs cv_bridge
)

utils_add_core_library()

# ===========================
# Executables
# ===========================
add_executable(write_kf_images
  src/write_keyframe_images.cpp
  src/ros/ros1_bag_image_reader.cpp
)
target_include_directories(write_kf_images SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_definitions(write_kf_images PRIVATE ROS_AVAILABLE=1)
target_link_libraries(write_kf_images ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
install(TARGETS write_kf_images
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Python tools and the ROS 1 / ROS 2 module they share
catkin_install_python(PROGRAMS
  scripts/brisk_feature_detection.py
  scripts/extract_bag_stereo.py
  scripts/extract_images.py
  scripts/gopro_combine_bags.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES scripts/ros_compat.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ===========================
# Install
# ===========================
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
