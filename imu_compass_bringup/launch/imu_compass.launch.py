import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    params_file = os.path.join(
        get_package_share_directory("imu_compass_bringup"),
        "config",
        "imu_compass.yaml",
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    return LaunchDescription(
        [
            use_sim_time_cmd,
            Node(
                package="imu_compass",
                executable="imu_compass_node",
                name="imu_compass_node",
                parameters=[configured_params],
                output="screen",
            ),
        ]
    )
