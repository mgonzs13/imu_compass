#ifndef IMU_COMPASS_HPP_
#define IMU_COMPASS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class IMUCompassNode : public rclcpp::Node {
public:
  IMUCompassNode();
  ~IMUCompassNode() = default;

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr decl_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_compass_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr debug_timer_;

  // Callback functions
  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
  void mag_callback(sensor_msgs::msg::MagneticField::SharedPtr msg);
  void decl_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void debug_callback();
  void
  repackage_imu_publish(const geometry_msgs::msg::TransformStamped &transform);

  // Heading Filter Methods and State
  void init_filter(double heading_meas);

  bool first_mag_reading_;
  bool first_gyro_reading_;
  bool filter_initialized_;
  bool gyro_update_complete_;

  std::string base_link_;

  double mag_zero_x_, mag_zero_y_, mag_zero_z_;

  sensor_msgs::msg::Imu curr_imu_reading_;

  // Heading Filter Variables
  double curr_heading_;
  double curr_heading_variance_;
  double sensor_timeout_;

  // Motion Update Variables
  double heading_prediction_;
  double heading_variance_prediction_;
  double heading_prediction_variance_;
  double mag_declination_;
  double last_motion_update_time_;
  double last_measurement_update_time_;

  // Measurement Update Variables
  double yaw_meas_variance_;
};

#endif // IMU_COMPASS_HPP_
