from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

import os,xacro

def generate_launch_description():
    
    pkg_path = get_package_share_directory('faucon_drone')
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    
    urdf = os.path.join(pkg_path, 'description_drone', 'drone.urdf.xacro')
    robot_desc = xacro.process_file(urdf).toxml()
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    
    bridge_config = os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml')
    
    
    ### Gazebo ######
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution([pkg_path, "worlds", "empty_gz.world"]),
            ],
        }.items(),
    )
    
    # PONT DE COMMUNICATION (ROS <-> GAZEBO)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{ 'config_file':bridge_config,
            
        }],
        output='screen'
    )
    
    # SPAWN DRONE
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'faucon_drone',
                   '-z', '2.5'],
        output='screen',
    )
    
    
    
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time
            }
        ],
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time
            }
        ],
    )
    
    rviz = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', use_rviz, '" == "true"'])),
    )
    
    
    
    
    
        
    
    return LaunchDescription(
        [
            
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value="true", 
                description="Use sim time if true"
            ),
            
            DeclareLaunchArgument(
                "use_rviz",
                 default_value="true",
                 description="Launch RViz if true ",
            ),
        
        robot_state_publisher,
        joint_state_publisher_node,
        rviz,
        spawn_drone,
        gz_sim,
        ros_gz_bridge,
            
        ]
    )