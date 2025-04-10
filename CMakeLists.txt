cmake_minimum_required(VERSION 3.5)
project(imu_compass)

# Default to C++14 or higher
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Executable
add_executable(imu_compass src/imu_compass.cpp)
ament_target_dependencies(imu_compass
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Installation
install(TARGETS imu_compass
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/compute_calibration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compute_calibration
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Package
ament_package()
