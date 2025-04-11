#include "imu_compass/imu_compass_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUCompassNode>());
  rclcpp::shutdown();
  return 0;
}
