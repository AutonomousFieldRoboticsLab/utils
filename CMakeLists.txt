cmake_minimum_required(VERSION 3.10)
project(utils_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# By default we build the ROS executables, but this can be disabled to build only the library
option(ENABLE_ROS "Build the ROS executables (ROS 1 or ROS 2, whichever is found)" ON)

# ===========================
# Dependencies shared by all builds
# ===========================
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ===========================
# ROS-agnostic core library
# ===========================
set(CORE_LIBRARY_SOURCES
  src/core/camera_intrinsics.cpp
  src/core/keyframe_writer.cpp
  src/core/trajectory.cpp
  src/utils/stamp_utils.cpp
)

# Called from ROS1.cmake / ROS2.cmake once the ROS package has been declared.
# The library is internal to the executables, so it is always static.
macro(utils_add_core_library)
  add_library(${PROJECT_NAME}_lib STATIC ${CORE_LIBRARY_SOURCES})
  target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
      src
      ${OpenCV_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS})
endmacro()

# ===========================
# Find our ROS version
# ===========================
# NOTE: Default to ROS 1 if both are in the environment
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if(catkin_FOUND AND ENABLE_ROS)
  message(STATUS "ROS *1* found, building ROS1.cmake")
  include(${PROJECT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif(ament_cmake_FOUND AND ENABLE_ROS)
  message(STATUS "ROS *2* found, building ROS2.cmake")
  include(${PROJECT_SOURCE_DIR}/cmake/ROS2.cmake)
else()
  message(WARNING "No ROS found (or ENABLE_ROS=OFF): building only the core library")
  utils_add_core_library()
endif()
