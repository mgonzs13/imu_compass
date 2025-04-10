#include "imu_compass/imu_compass.hpp"

#include "tf2/utils.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

double magn(tf2::Vector3 a) {
  return std::sqrt(a.x() * a.x() + a.y() * a.y() + a.z() * a.z());
}

IMUCompass::IMUCompass() : Node("imu_compass") {
  // Declare and get parameters
  this->declare_parameter("mag_bias.x", 0.0);
  this->declare_parameter("mag_bias.y", 0.0);
  this->declare_parameter("mag_bias.z", 0.0);
  this->declare_parameter("compass.sensor_timeout", 0.5);
  this->declare_parameter("compass.yaw_meas_variance", 10.0);
  this->declare_parameter("compass.gyro_meas_variance", 0.01);
  this->declare_parameter("compass.mag_declination", 0.0);

  this->get_parameter("mag_bias.x", mag_zero_x_);
  this->get_parameter("mag_bias.y", mag_zero_y_);
  this->get_parameter("mag_bias.z", mag_zero_z_);
  this->get_parameter("compass.sensor_timeout", sensor_timeout_);
  this->get_parameter("compass.yaw_meas_variance", yaw_meas_variance_);
  this->get_parameter("compass.gyro_meas_variance",
                      heading_prediction_variance_);
  this->get_parameter("compass.mag_declination", mag_declination_);

  RCLCPP_INFO(this->get_logger(), "Using magnetometer bias (x,y): %f,%f",
              mag_zero_x_, mag_zero_y_);
  RCLCPP_INFO(this->get_logger(), "Using variance %f", yaw_meas_variance_);
  RCLCPP_INFO(this->get_logger(), "Using magnetic declination %f (%f deg)",
              mag_declination_, mag_declination_ * 180 / M_PI);

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&IMUCompass::imuCallback, this, _1));
  mag_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "imu/mag", 10, std::bind(&IMUCompass::magCallback, this, _1));
  decl_sub_ = create_subscription<std_msgs::msg::Float32>(
      "imu/declination", 10, std::bind(&IMUCompass::declCallback, this, _1));

  // Publishers
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_compass", 10);
  compass_pub_ =
      create_publisher<std_msgs::msg::Float32>("imu/compass_heading", 10);
  mag_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/mag_calib", 10);
  raw_compass_pub_ =
      create_publisher<std_msgs::msg::Float32>("imu/raw_compass_heading", 10);

  // Timer
  debug_timer_ = create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&IMUCompass::debugCallback, this));

  first_mag_reading_ = false;
  first_gyro_reading_ = false;
  gyro_update_complete_ = false;
  filter_initialized_ = false;

  last_motion_update_time_ = now().seconds();

  RCLCPP_INFO(this->get_logger(), "Compass Estimator Started");
}

void IMUCompass::debugCallback() {
  if (!first_gyro_reading_)
    RCLCPP_WARN(this->get_logger(),
                "Waiting for IMU data, no gyroscope data available");

  if (!first_mag_reading_)
    RCLCPP_WARN(this->get_logger(),
                "Waiting for mag data, no magnetometer data "
                "available, Filter not initialized");

  double now_sec = now().seconds();

  if ((now_sec - last_motion_update_time_) > sensor_timeout_ &&
      first_gyro_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Gyroscope data being received too slow or not at all");
    first_gyro_reading_ = false;
  }

  if ((now_sec - last_measurement_update_time_) > sensor_timeout_ &&
      first_mag_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Magnetometer data being received too slow or not at all");
    first_mag_reading_ = false;
    filter_initialized_ = false;
  }
}

void IMUCompass::imuCallback(sensor_msgs::msg::Imu::SharedPtr data) {
  auto gyro_vector = data->angular_velocity;
  geometry_msgs::msg::Vector3 gyro_vector_transformed;

  if (!first_gyro_reading_)
    first_gyro_reading_ = true;

  double dt = now().seconds() - last_motion_update_time_;
  last_motion_update_time_ = now().seconds();

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("base_link", data->header.frame_id,
                                            tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }

  tf2::Vector3 orig;
  tf2::fromMsg(gyro_vector, orig);
  tf2::Matrix3x3 rot(tf2::Quaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w));

  auto transformed = rot * orig;
  gyro_vector_transformed = tf2::toMsg(transformed);

  double yaw_gyro = gyro_vector_transformed.z;

  if (filter_initialized_) {
    heading_prediction_ = curr_heading_ + yaw_gyro * dt;
    heading_variance_prediction_ =
        curr_heading_variance_ + heading_prediction_variance_;

    if (heading_prediction_ > M_PI)
      heading_prediction_ -= 2 * M_PI;
    if (heading_prediction_ < -M_PI)
      heading_prediction_ += 2 * M_PI;

    gyro_update_complete_ = true;
  }

  curr_imu_reading_ = *data;
}

void IMUCompass::declCallback(const std_msgs::msg::Float32::SharedPtr data) {
  mag_declination_ = data->data;
  RCLCPP_INFO(this->get_logger(), "Updated declination: %f (%f deg)",
              mag_declination_, mag_declination_ * 180 / M_PI);
}

void IMUCompass::magCallback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr data) {
  if (std::isnan(data->vector.x) || std::isnan(data->vector.y) ||
      std::isnan(data->vector.z))
    return;

  double now_sec = now().seconds();
  last_measurement_update_time_ = now_sec;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("base_link", data->header.frame_id,
                                            tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }

  tf2::Vector3 mag_vec(data->vector.x, data->vector.y, data->vector.z);
  tf2::Matrix3x3 rot(tf2::Quaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w));

  tf2::Vector3 transformed = rot * mag_vec;

  double mag_x = transformed.x() - mag_zero_x_;
  double mag_y = transformed.y() - mag_zero_y_;
  double mag_z = transformed.z();

  tf2::Vector3 calib_mag(mag_x, mag_y, mag_z);
  calib_mag = calib_mag / magn(calib_mag);

  geometry_msgs::msg::Vector3Stamped calibrated_msg;
  calibrated_msg.header.stamp = now();
  calibrated_msg.header.frame_id = "imu_link";
  calibrated_msg.vector = tf2::toMsg(calib_mag);
  mag_pub_->publish(calibrated_msg);

  tf2::Quaternion q;
  tf2::fromMsg(curr_imu_reading_.orientation, q);
  tf2::Transform imu_meas(q);
  tf2::Quaternion trans_q(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
  imu_meas = imu_meas * tf2::Transform(trans_q);

  double roll, pitch, yaw;
  tf2::Matrix3x3(imu_meas.getRotation()).getRPY(roll, pitch, yaw);

  double t_mag_x = calib_mag.x() * cos(pitch) + calib_mag.z() * sin(pitch);
  double t_mag_y = calib_mag.x() * sin(roll) * sin(pitch) +
                   calib_mag.y() * cos(roll) -
                   calib_mag.z() * sin(roll) * cos(pitch);
  double heading_meas = atan2(t_mag_x, t_mag_y);

  if (!first_mag_reading_) {
    initFilter(heading_meas);
    first_mag_reading_ = true;
    return;
  }

  if (gyro_update_complete_) {
    double kalman_gain = heading_variance_prediction_ /
                         (heading_variance_prediction_ + yaw_meas_variance_);
    double innovation = heading_meas - heading_prediction_;

    if (std::abs(innovation) > M_PI)
      curr_heading_ = heading_meas;
    else
      curr_heading_ = heading_prediction_ + kalman_gain * innovation;

    curr_heading_variance_ = (1 - kalman_gain) * heading_variance_prediction_;

    std_msgs::msg::Float32 raw_heading;
    raw_heading.data = heading_meas;
    raw_compass_pub_->publish(raw_heading);

    repackageImuPublish(transform);
    gyro_update_complete_ = false;
  }
}

void IMUCompass::repackageImuPublish(
    const geometry_msgs::msg::TransformStamped &transform) {
  tf2::Quaternion q;
  tf2::fromMsg(curr_imu_reading_.orientation, q);
  tf2::Quaternion trans_q(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
  tf2::Transform tf_imu(q), tf_trans(trans_q);
  tf2::Transform tf_combined = tf_imu * tf_trans;

  double compass_heading = curr_heading_ - mag_declination_;
  tf2::Quaternion new_yaw = tf2::Quaternion();
  new_yaw.setRPY(0, 0, compass_heading);
  tf2::Quaternion diff = tf2::Quaternion();
  diff.setRPY(0, 0, compass_heading - tf2::getYaw(q));

  tf2::Quaternion corrected = diff * q;
  tf2::Transform corrected_tf(corrected);
  tf2::Transform back_transform = corrected_tf * tf_trans.inverse();

  curr_imu_reading_.orientation = tf2::toMsg(back_transform.getRotation());

  std_msgs::msg::Float32 heading_msg;
  heading_msg.data = compass_heading;
  compass_pub_->publish(heading_msg);
  imu_pub_->publish(curr_imu_reading_);
}

void IMUCompass::initFilter(double heading_meas) {
  curr_heading_ = heading_meas;
  curr_heading_variance_ = 1.0;
  filter_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Compass filter initialized with heading: %f",
              heading_meas);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUCompass>());
  rclcpp::shutdown();
  return 0;
}
