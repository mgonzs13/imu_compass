cmake_minimum_required(VERSION 3.5)
project(imu_compass_bringup)

# Dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Package
ament_package()
