from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "faucon_localisation")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            # Doit démarrer en premier : publie /datum et /gnss/datum dès le premier fix GNSS
            launch_ros.actions.Node(
                package="faucon_localisation",
                executable="datum_manager.py",
                name="datum_manager",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "gnss/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
        ]
    )