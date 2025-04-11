
#include "tf2/utils.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "imu_compass/imu_compass_node.hpp"

using std::placeholders::_1;

double magn(tf2::Vector3 a) {
  return std::sqrt(a.x() * a.x() + a.y() * a.y() + a.z() * a.z());
}

IMUCompassNode::IMUCompassNode() : Node("imu_compass_node") {

  // Declare and get parameters
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("mag_bias.x", 0.0);
  this->declare_parameter("mag_bias.y", 0.0);
  this->declare_parameter("mag_bias.z", 0.0);
  this->declare_parameter("compass.sensor_timeout", 0.5);
  this->declare_parameter("compass.yaw_meas_variance", 10.0);
  this->declare_parameter("compass.gyro_meas_variance", 0.01);
  this->declare_parameter("compass.mag_declination", 0.0);

  this->get_parameter("base_link_frame", this->base_link_);
  this->get_parameter("mag_bias.x", this->mag_zero_x_);
  this->get_parameter("mag_bias.y", this->mag_zero_y_);
  this->get_parameter("mag_bias.z", this->mag_zero_z_);
  this->get_parameter("compass.sensor_timeout", this->sensor_timeout_);
  this->get_parameter("compass.yaw_meas_variance", this->yaw_meas_variance_);
  this->get_parameter("compass.gyro_meas_variance",
                      this->heading_prediction_variance_);
  this->get_parameter("compass.mag_declination", this->mag_declination_);

  RCLCPP_INFO(this->get_logger(), "Using magnetometer bias (x,y): %f,%f",
              this->mag_zero_x_, this->mag_zero_y_);
  RCLCPP_INFO(this->get_logger(), "Using variance %f",
              this->yaw_meas_variance_);
  RCLCPP_INFO(this->get_logger(), "Using magnetic declination %f (%f deg)",
              this->mag_declination_, this->mag_declination_ * 180 / M_PI);

  // TF setup
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Subscribers
  this->imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(),
      std::bind(&IMUCompassNode::imu_callback, this, _1));
  this->mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "imu/mag", rclcpp::SensorDataQoS(),
      std::bind(&IMUCompassNode::mag_callback, this, _1));
  this->decl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "imu/declination", rclcpp::SensorDataQoS(),
      std::bind(&IMUCompassNode::decl_callback, this, _1));

  // Publishers
  this->imu_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>("imu/data_compass", 10);
  this->compass_pub_ =
      create_publisher<std_msgs::msg::Float32>("imu/compass_heading", 10);
  this->mag_pub_ =
      create_publisher<sensor_msgs::msg::MagneticField>("imu/mag_calib", 10);
  this->raw_compass_pub_ =
      create_publisher<std_msgs::msg::Float32>("imu/raw_compass_heading", 10);

  // Timer
  this->debug_timer_ =
      this->create_wall_timer(std::chrono::seconds(1),
                              std::bind(&IMUCompassNode::debug_callback, this));

  this->first_mag_reading_ = false;
  this->first_gyro_reading_ = false;
  this->gyro_update_complete_ = false;
  this->filter_initialized_ = false;

  this->last_motion_update_time_ = now().seconds();

  RCLCPP_INFO(this->get_logger(), "Compass Estimator Started");
}

void IMUCompassNode::debug_callback() {

  if (!this->first_gyro_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Waiting for IMU data, no gyroscope data available");
  }

  if (!this->first_mag_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Waiting for mag data, no magnetometer data "
                "available, Filter not initialized");
  }

  double now_sec = now().seconds();

  if ((now_sec - this->last_motion_update_time_) > this->sensor_timeout_ &&
      this->first_gyro_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Gyroscope data being received too slow or not at all");
    this->first_gyro_reading_ = false;
  }

  if ((now_sec - this->last_measurement_update_time_) > this->sensor_timeout_ &&
      this->first_mag_reading_) {
    RCLCPP_WARN(this->get_logger(),
                "Magnetometer data being received too slow or not at all");
    this->first_mag_reading_ = false;
    this->filter_initialized_ = false;
  }
}

void IMUCompassNode::imu_callback(sensor_msgs::msg::Imu::SharedPtr data) {

  auto gyro_vector = data->angular_velocity;
  geometry_msgs::msg::Vector3 gyro_vector_transformed;

  if (!this->first_gyro_reading_) {
    first_gyro_reading_ = true;
  }

  double dt = now().seconds() - this->last_motion_update_time_;
  this->last_motion_update_time_ = now().seconds();

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = this->tf_buffer_->lookupTransform(
        this->base_link_, data->header.frame_id, tf2::TimePointZero);
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

  if (this->filter_initialized_) {
    this->heading_prediction_ = this->curr_heading_ + yaw_gyro * dt;
    this->heading_variance_prediction_ =
        this->curr_heading_variance_ + this->heading_prediction_variance_;

    if (this->heading_prediction_ > M_PI)
      this->heading_prediction_ -= 2 * M_PI;
    if (this->heading_prediction_ < -M_PI)
      this->heading_prediction_ += 2 * M_PI;

    this->gyro_update_complete_ = true;
  }

  this->curr_imu_reading_ = *data;
}

void IMUCompassNode::decl_callback(
    const std_msgs::msg::Float32::SharedPtr data) {
  this->mag_declination_ = data->data;
  RCLCPP_INFO(this->get_logger(), "Updated declination: %f (%f deg)",
              this->mag_declination_, this->mag_declination_ * 180 / M_PI);
}

void IMUCompassNode::mag_callback(
    sensor_msgs::msg::MagneticField::SharedPtr data) {

  if (std::isnan(data->magnetic_field.x) ||
      std::isnan(data->magnetic_field.y) || std::isnan(data->magnetic_field.z))
    return;

  double now_sec = now().seconds();
  this->last_measurement_update_time_ = now_sec;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = this->tf_buffer_->lookupTransform(
        this->base_link_, data->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }

  tf2::Vector3 mag_vec(data->magnetic_field.x, data->magnetic_field.y,
                       data->magnetic_field.z);
  tf2::Matrix3x3 rot(tf2::Quaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w));

  tf2::Vector3 transformed = rot * mag_vec;

  double mag_x = transformed.x() - mag_zero_x_;
  double mag_y = transformed.y() - mag_zero_y_;
  double mag_z = transformed.z();

  tf2::Vector3 calib_mag(mag_x, mag_y, mag_z);
  calib_mag = calib_mag / magn(calib_mag);

  sensor_msgs::msg::MagneticField calibrated_msg;
  calibrated_msg.header.stamp = now();
  calibrated_msg.header.frame_id = "imu_link";
  calibrated_msg.magnetic_field.x = mag_x;
  calibrated_msg.magnetic_field.y = mag_y;
  calibrated_msg.magnetic_field.z = mag_z;
  this->mag_pub_->publish(calibrated_msg);

  tf2::Quaternion q;
  tf2::fromMsg(this->curr_imu_reading_.orientation, q);
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

  if (!this->first_mag_reading_) {
    this->init_filter(heading_meas);
    this->first_mag_reading_ = true;
    return;
  }

  if (this->gyro_update_complete_) {
    double kalman_gain =
        this->heading_variance_prediction_ /
        (this->heading_variance_prediction_ + this->yaw_meas_variance_);
    double innovation = heading_meas - this->heading_prediction_;

    if (std::abs(innovation) > M_PI)
      this->curr_heading_ = heading_meas;
    else
      this->curr_heading_ =
          this->heading_prediction_ + kalman_gain * innovation;

    this->curr_heading_variance_ =
        (1 - kalman_gain) * this->heading_variance_prediction_;

    std_msgs::msg::Float32 raw_heading;
    raw_heading.data = heading_meas;
    this->raw_compass_pub_->publish(raw_heading);

    this->repackage_imu_publish(transform);
    this->gyro_update_complete_ = false;
  }
}

void IMUCompassNode::repackage_imu_publish(
    const geometry_msgs::msg::TransformStamped &transform) {

  tf2::Quaternion q;
  tf2::fromMsg(this->curr_imu_reading_.orientation, q);

  tf2::Quaternion trans_q(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
  tf2::Transform tf_imu(q), tf_trans(trans_q);
  tf2::Transform tf_combined = tf_imu * tf_trans;

  double compass_heading = this->curr_heading_ - this->mag_declination_;
  tf2::Quaternion new_yaw = tf2::Quaternion();
  new_yaw.setRPY(0, 0, compass_heading);

  double roll, pitch, _;
  tf2::Matrix3x3(q).getRPY(roll, pitch, _);

  tf2::Quaternion new_orientation;
  new_orientation.setRPY(roll, pitch, compass_heading);

  this->curr_imu_reading_.orientation = tf2::toMsg(new_orientation);

  std_msgs::msg::Float32 heading_msg;
  heading_msg.data = compass_heading;
  this->compass_pub_->publish(heading_msg);
  this->imu_pub_->publish(this->curr_imu_reading_);
}

void IMUCompassNode::init_filter(double heading_meas) {
  this->curr_heading_ = heading_meas;
  this->curr_heading_variance_ = 0.1;
  this->filter_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Compass filter initialized with heading: %f",
              heading_meas);
}
