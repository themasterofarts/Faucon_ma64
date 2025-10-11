from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition



def generate_launch_description():
    pkg_path = get_package_share_directory("base_desc")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_maize_field = get_package_share_directory("virtual_maize_field")
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()

    twist_mux_params = os.path.join(get_package_share_directory('base_desc'),'config','twrist_mux.yaml')


    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([pkg_path, "worlds", "empty_gz.world"])
        }.items(),
    )

    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_maize_field, "launch", "simulation.launch.py")
        ),
        
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen")

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "bot",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
            "-R",
            "0.0",
            "-P",
            "0.0",
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_path, "config", "bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )

   
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time, 'use_stamped': use_ros2_control}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
  

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use sim time if true'),
            DeclareLaunchArgument('use_ros2_control', default_value='true',
                              description='ROS2 control enabled if true'),

            robot_state_publisher,
            joint_state_publisher_gui,
            #joint_state_publisher_node,
            rviz,
            spawn_robot,
            gz_bridge,
            gz_sim,
            #launch_world,
            ros_gz_image_bridge,

            diff_drive_spawner,
            joint_broad_spawner,
            twist_mux
            
        ]
    )
