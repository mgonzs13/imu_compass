# imu_compass

A ROS 2 package that fuses data from an Inertial Measurement Unit (IMU) and a magnetometer to provide a more accurate and stable heading estimation for robotic applications.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [ROS 2 Node: `imu_compass`](#ros-2-node-imu_compass)
  - [Node Description](#node-description)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
  - [Parameters](#parameters)
- [Launch File: `imu_compass_bringup`](#launch-file-imu_compass_bringup)
  - [Launch File: `imu_compasslaunchpy`](#launch-file-imu_compasslaunchpy)
  - [Usage](#usage)
  - [Parameters](#parameters-1)
- [Installation](#installation)
- [Usage](#usage-1)
- [Magnetometer Calibration](#magnetometer-calibration)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Overview

The `imu_compass` node combines gyroscope and magnetometer data to compute a reliable heading. This fusion mitigates the drift commonly associated with gyroscopes and the noise inherent in magnetometers, resulting in a cleaner heading estimate suitable for navigation and localization tasks.

## Features

- **Sensor Fusion**: Integrates IMU and magnetometer data for enhanced heading accuracy.
- **Bias Correction**: Applies magnetometer bias corrections to account for sensor offsets.
- **Magnetic Declination**: Incorporates magnetic declination adjustments based on geographic location.
- **ROS 2 Integration**: Provides ROS 2 topics for integration into robotic systems.

## ROS 2 Node: `imu_compass`

### Node Description

The `imu_compass` node subscribes to raw IMU and magnetometer data, processes this information, and publishes the fused heading estimate. It also handles bias correction and declination adjustments.

### Subscribed Topics

- `/imu/data` (`sensor_msgs/msg/Imu`): Raw IMU data including angular velocity and linear acceleration.
- `/imu/mag` (`sensor_msgs/msg/MagneticField`): Raw magnetometer data.
- `/imu/declination` (`std_msgs/msg/Float32`): Optional magnetic declination value.

### Published Topics

- `/imu/data_compass` (`sensor_msgs/msg/Imu`): IMU data with corrected orientation.
- `/imu/compass_heading` (`std_msgs/msg/Float32`): Fused heading estimate.
- `/imu/mag_calib` (`geometry_msgs/msg/Vector3Stamped`): Calibrated magnetometer data.
- `/imu/raw_compass_heading` (`std_msgs/msg/Float32`): Raw heading computed directly from magnetometer data.

### Parameters

- `base_link_frame` (str, default: base_link): Reference frame to create the final heading.
- `mag_bias/x` (float, default: 0.0): Magnetometer bias in the X-axis.
- `mag_bias/y` (float, default: 0.0): Magnetometer bias in the Y-axis.
- `mag_bias/z` (float, default: 0.0): Magnetometer bias in the Z-axis.
- `compass/sensor_timeout` (float, default: 0.5): Timeout in seconds to consider sensor data as stale.
- `compass/yaw_meas_variance` (float, default: 10.0): Variance of the yaw measurement.
- `compass/gyro_meas_variance` (float, default: 0.01): Variance of the gyroscope measurement.
- `compass/mag_declination` (float, default: 0.0): Magnetic declination in radians.

## Launch File: `imu_compass_bringup`

The `imu_compass_bringup` package provides a launch file to start the `imu_compass` node with predefined parameters.

### Launch File: `imu_compass.launch.py`

This launch file initializes the `imu_compass` node with parameters specified in a YAML configuration file.

#### Usage

```bash
ros2 launch imu_compass_bringup imu_compass.launch.py
```

#### Parameters

The launch file references a YAML configuration file where you can set the parameters mentioned above. Ensure that the paths to your sensor topics and any required transformations are correctly specified.

## Installation

1. Clone the repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/mgonzs13/imu_compass.git
   ```

2. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select imu_compass imu_compass_bringup
   ```

3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

After installation, you can launch the node using the provided launch file:

```bash
ros2 launch imu_compass_bringup imu_compass.launch.py
```

Ensure that your IMU and magnetometer are publishing data to the expected topics.

## Magnetometer Calibration

The `compute_calibration.py` script performs magnetometer bias calibration from a ROS 2 bag file. Below are the available command-line arguments:

| Argument      | Type  | Default                 | Description                                                                                              |
| ------------- | ----- | ----------------------- | -------------------------------------------------------------------------------------------------------- |
| `bag`         | `str` | —                       | **(Required)** Path to the input ROS 2 bag directory.                                                    |
| `outfile`     | `str` | `/tmp/calibration.yaml` | Optional output file path where the calibration result (bias and radius) will be written in YAML format. |
| `--imu-topic` | `str` | `/imu/data`             | IMU topic name from the bag file, used to extract orientation data.                                      |
| `--mag-topic` | `str` | `/imu/mag`              | Magnetometer topic name from the bag file, used to extract magnetic field data.                          |
| `--plots`     | flag  | `False`                 | If set, enables display and saving of diagnostic plots (yaw and 3D magnetic points).                     |

### How It Works

The script listens to magnetometer data over a specified ROS 2 topic, collects samples as you move the IMU/magnetometer in various orientations, and then computes the center of the measured magnetic field values as the bias.

### Usage

1. Record a bag, that contains imu and mag data, using `ros2 bag record`.
2. Run the script:

   ```bash
   python3 compute_calibration.py <path-to-your-bag>
   ```

3. The script will save the bias values for the X, Y, and Z axes in a yaml file, by default will be at `/tmp/calibration.yaml`.

### Output Example

The output will be a yaml file with the resulting estimated bias values. For example, you can a resulting file as the follows:

```yaml
mag_bias:
  x: -0.213
  y: 0.127
  z: 0.045
```

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This package is based on the original work by Clearpath Robotics, adapted for ROS 2 compatibility.
