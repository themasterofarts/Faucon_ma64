import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = "faucon_navigation"
    
    
    ########"#### chemin du fichier navigation bringup_alleger #################
    new_navigation_bringup = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        "navigation_bring_launch_alleger.py")

   
    nav2_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "fauncon_nav2_params.yaml"
    )

    bringup_dir = get_package_share_directory("nav2_bringup")

    launch_dir = os.path.join(bringup_dir, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",        
            "0.0", "0.0", "0.0",  
            "map",
            "odom",
        ],
    )


    static_tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",  
            "0.0", "0.0", "0.0",  
            "odom",
            "base_link",
        ],
    )


    # Launch them all!
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            launch.actions.DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    bringup_dir, "rviz", "nav2_default_view.rviz"
                ),
                description="Full path to the RVIZ config file to use",
            ),
            
            launch.actions.DeclareLaunchArgument(
                name="params_file",
                default_value=nav2_params_path,
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            launch.actions.DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
           
            # Launch the ROS 2 Navigation Stack
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    new_navigation_bringup  
                    #os.path.join(launch_dir, "bringup_launch.py"): décommenté  si voulez utiliser le  fichier bringup présent sur le github de nav2
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_config_file,
                }.items(),
            ),
            static_tf,
            #static_tf2,
        ]
    )
