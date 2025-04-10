#!/usr/bin/env python3

import datetime
import argparse
import numpy as np

from scipy import optimize

try:
    from matplotlib import pyplot as plt
except ImportError:
    plt = None

from tf_transformations import quaternion_matrix

import rosbag2_py

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
        (topic, data, t) = reader.read_next()
        if topic in topic_filter:
            messages[topic].append((data, t))

    return messages


bag_data = read_ros2_messages(args.bag, ["/imu/rpy", "/imu/mag"])

# Static transform assumed if no TF info
imu_rot = np.array([0, 0, 0, 1])
t_mat = np.linalg.inv(quaternion_matrix(imu_rot))

# Extract RPY (yaw)
time_yaw_tuples = []
for data, t in bag_data.get("/imu/rpy", []):
    # Simulate the message (this part would require actual deserialization)
    # We'll assume data is a dict for now (replace with proper decoding if needed)
    msg = eval(data.decode())  # WARNING: Replace with actual ROS 2 deserialization
    vec = np.array([msg["vector"]["x"], msg["vector"]["y"], msg["vector"]["z"], 0])
    transformed = t_mat.dot(vec)
    time_yaw_tuples.append((t / 1e9, float(transformed[2])))

if len(time_yaw_tuples) < 100:
    print("Insufficient data or missing /imu/rpy topic.")
    exit(1)

time_yaw = list(zip(*time_yaw_tuples))
yaw_diff = np.diff(time_yaw[-1])
filtered = [tup for tup in zip(*time_yaw) if abs(tup[-1]) < 3.0]
smoothed = np.convolve(list(zip(*filtered))[-1], [1 / 30.0] * 30, mode="same")

time_start = filtered[50][0]
time_end = filtered[-50][0]

# Plot RPY
if plt and args.plots:
    plt.figure()
    plt.subplot(211)
    plt.scatter(np.array(time_yaw[0]), smoothed)
    plt.axvline(time_start)
    plt.axvline(time_end)

# Collect magnetometer data
vecs = []
for data, t in bag_data.get("/imu/mag", []):
    t_sec = t / 1e9
    if time_start <= t_sec <= time_end:
        msg = eval(data.decode())  # Replace with proper ROS 2 message deserialization
        vec = np.array([msg["vector"]["x"], msg["vector"]["y"], msg["vector"]["z"], 0])
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
    plt.subplot(212, projection="3d")
    plt.scatter(x, y, z)
    plt.title("Magnetic Field Points")
    plt.show()
