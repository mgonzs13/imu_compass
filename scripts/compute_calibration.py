#!/usr/bin/env python3

import datetime
import argparse
import numpy as np

from scipy import optimize
from matplotlib import pyplot as plt

from tf_transformations import quaternion_matrix

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

parser = argparse.ArgumentParser(
    description="Process ROS 2 bag file for compass calibration."
)
parser.add_argument("bag", type=str, help="Input ROS 2 bag directory")
parser.add_argument(
    "outfile",
    type=str,
    nargs="?",
    default="/tmp/calibration.yaml",
    help="Output YAML file",
)
parser.add_argument(
    "--imu-topic",
    type=str,
    default="/imu/data",
    help="IMU topic name",
)
parser.add_argument(
    "--mag-topic",
    type=str,
    default="/imu/mag",
    help="Magnetometer topic name",
)
parser.add_argument(
    "--plots", action="store_true", help="Show plots if matplotlib is available"
)
args = parser.parse_args()


# Helper to read messages from ROS 2 bag
def read_ros2_messages(bag_path, topic_filter):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    type_map = {x.name: x.type for x in reader.get_all_topics_and_types()}

    topic_types = {}
    for topic in topic_filter:
        if topic in type_map:
            topic_types[topic] = type_map[topic]

    messages = {topic: [] for topic in topic_filter}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in topic_filter:
            messages[topic].append((data, t))

    return messages, type_map


bag_data, type_map = read_ros2_messages(args.bag, [args.imu_topic, args.mag_topic])

# Load message types
imu_msg_type = get_message(type_map[args.imu_topic])
mag_msg_type = get_message(type_map[args.mag_topic])

# Static transform assumed if no TF info
imu_rot = np.array([0, 0, 0, 1])
t_mat = np.linalg.inv(quaternion_matrix(imu_rot))

# Extract RPY (yaw)
time_yaw_tuples = []
for data, t in bag_data.get(args.imu_topic, []):
    msg = deserialize_message(data, imu_msg_type)
    vec = np.array(
        [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            0,
        ]
    )
    transformed = t_mat.dot(vec)
    time_yaw_tuples.append((t / 1e9, float(transformed[2])))

if len(time_yaw_tuples) < 100:
    print("Insufficient data or missing IMU topic.")
    exit(1)

time_yaw = list(zip(*time_yaw_tuples))
yaw_diff = np.diff(time_yaw[-1])
filtered = [tup for tup in zip(*time_yaw) if abs(tup[-1]) < 3.0]
smoothed = np.convolve(list(zip(*filtered))[-1], [1 / 30.0] * 30, mode="same")

if len(filtered) < 10:
    print(f"Not enough filtered samples ({len(filtered)} found) to compute calibration.")
    exit(1)

# Use central 10% to 90% portion
start_idx = int(len(filtered) * 0.1)
end_idx = int(len(filtered) * 0.9)

time_start = filtered[start_idx][0]
time_end = filtered[end_idx][0]

# Plot RPY
if plt and args.plots:
    plt.figure()
    plt.subplot(211)
    plt.scatter(np.array(time_yaw[0]), smoothed)
    plt.axvline(time_start)
    plt.axvline(time_end)

# Collect magnetometer data
vecs = []
for data, t in bag_data.get(args.mag_topic, []):
    t_sec = t / 1e9
    if time_start <= t_sec <= time_end:
        msg = deserialize_message(data, mag_msg_type)
        vec = np.array(
            [
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z,
                0,
            ]
        )
        transformed = t_mat.dot(vec)
        vecs.append(transformed[:3])

vecs = np.array(vecs)
print(f"Using {len(vecs)} magnetometer samples.")

# Fit circle to XY plane
x, y, z = vecs[:, 0], vecs[:, 1], vecs[:, 2]


def calc_R(xc, yc):
    return np.hypot(x - xc, y - yc)


def f_2(c):
    return calc_R(*c) - calc_R(*c).mean()


center_est = (x.mean(), y.mean())
center, _ = optimize.leastsq(f_2, center_est)
radius = calc_R(*center).mean()
z_center = z.mean()

print(
    f"Magnetic circle centered at ({center[0]:.3f}, {center[1]:.3f}, {z_center:.3f}) with radius {radius:.3f}"
)

# Write YAML
with open(args.outfile, "w") as f:
    f.write(f"# Generated from {args.bag} on {datetime.date.today()}\n")
    f.write("mag_bias:\n")
    f.write(f"  x: {center[0]}\n")
    f.write(f"  y: {center[1]}\n")
    f.write(f"  z: {z_center}\n")
    f.write(f"  radius: {radius}\n")

print(f"Calibration written to {args.outfile}")

# 3D plot
if plt and args.plots:
    fig = plt.figure(figsize=(10, 6))

    # Plot smoothed yaw
    ax1 = fig.add_subplot(211)
    ax1.scatter(np.array(time_yaw[0]), smoothed)
    ax1.axvline(time_start, color="r", linestyle="--")
    ax1.axvline(time_end, color="r", linestyle="--")
    ax1.set_title("Smoothed Yaw Over Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Yaw (rad)")

    # Attempt 3D plot without importing Axes3D directly
    try:
        ax2 = fig.add_subplot(212, projection="3d")
        ax2.scatter(x, y, z)
        ax2.set_title("Magnetic Field Points")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.set_zlabel("Z")
    except Exception as e:
        print(f"Could not plot 3D graph: {e}")
        ax2 = fig.add_subplot(212)
        ax2.scatter(x, y)
        ax2.set_title("Magnetic Field Points (XY only)")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")

    plt.tight_layout()
    plt.savefig("calibration_plot.png")
    print("Plot saved to calibration_plot.png")
