import os
import launch
from launch  import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,command

from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    
    package_name = "faucon_navigation"
    
    
    nav2_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "fauncon_nav2_params.yaml"
    )
    
    bringup_dir = get_package_share_directory("nav2_bringup")
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    
    
    
    
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
                "params_file",
                default_value= nav2_params_path,
                description = "chemin vers le dossier de configuration"
            ),
            
            launch.actions.DeclareLaunchArgument(
                "autostart",
                default_value= "True",
                description = "lancement automatique de la pile nav2"
            ),
            
            
            #####   lancement de fichier de la pile de navigation nav2    ###########
            
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir,
                             "launch",
                             "navigation_launch.py")
                ),
                
                launch_arguments= {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart")
                }.items()              
            ),
            
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_config" : rviz_config_file
                }.items()
            ),
            static_tf
        ]
    )