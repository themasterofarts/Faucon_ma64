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
    
    

   
    nav2_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "fauncon_nav2_params.yaml"
    )
    
    

    bringup_dir = get_package_share_directory("nav2_bringup")
    localization_dir = get_package_share_directory("faucon_localisation")

    launch_dir = os.path.join(bringup_dir, "launch")
    launch_dir_localization = os.path.join(
        localization_dir, "launch"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "-2.28", "-3.83", "0.0",        
            "-0.01", "-0.03", "-1.52",  
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
            "0.0", "0.0", "-1.52",  
            "odom",
            "base_link",
        ],
    )

    # mission manager
    mission_manager_cmd = Node(
        package="faucon_navigation",
        executable="mission_manager.py",
        name="mission_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    #robot localization nodes 
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_localization, 'dual_ekf_navsat.launch.py'))
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
                   os.path.join(launch_dir, "bringup_launch.py")  
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
            #static_tf,
            #static_tf2,
            robot_localization_cmd,
            mission_manager_cmd,
        ]
    )
