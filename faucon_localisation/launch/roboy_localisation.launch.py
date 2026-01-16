import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "faucon_localisation"

    # Chemin du package
    pkg_path = get_package_share_directory(package_name)

    # Chemin du fichier EKF
    ekf_params_path = os.path.join(
        pkg_path,
        "config",
        "ekf_node.yaml"
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_config_file = LaunchConfiguration("ekf_config_file")

    # Node robot_localization
    robot_localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),

        DeclareLaunchArgument(
            "ekf_config_file",
            default_value=ekf_params_path,
            description="Full path to EKF yaml file"
        ),

        robot_localisation_node
    ])
