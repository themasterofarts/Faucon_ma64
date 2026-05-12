from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from os import path


def launch_setup(context, *args, **kwargs):
   
    pkg_path = get_package_share_directory("faucon_base_desc")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_maize_field = get_package_share_directory("virtual_maize_field")
    pkg_faucon_ihm = get_package_share_directory("faucon_ihm")  
    
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    xacro_file_mini = os.path.join(pkg_path, "description_mini", "robot.urdf.xacro")
    
   
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mini_value = LaunchConfiguration("use_mini").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    
    
    if use_mini_value.lower() == "true":
        robot_desc = xacro.process_file(xacro_file_mini).toxml()
        spawn_z = "0.4"
        spawn_y = "-2.94"
        use_ros2_control_value = "false"
    else:
        robot_desc = xacro.process_file(xacro_file).toxml()
        spawn_z = "1.0"
        spawn_y = "-5.326" #-2.83
        use_ros2_control_value = LaunchConfiguration("use_ros2_control").perform(context)

    twist_mux_params = os.path.join(
        get_package_share_directory("faucon_base_desc"), "config", "twist_mux.yaml"
    )

   
    environment = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        path.join(get_package_share_directory("faucon_base_desc"), "worlds"),
    )

    
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

   
    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_maize_field, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world_path": os.path.join(pkg_path, "worlds", "virtual_maize_field"),
            "world_name": "straight_rows_debris.world",  #straight_rows_debris  generated mon_champ
            "headless": headless,
            "verbose": verbose,
        }.items(),
    )

    faucon_ihm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_faucon_ihm, "launch", "ihm_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
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

   
    control_4ws = Node(
        package="faucon_control",
        executable="control_4ws.py",
        name="control_4ws",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    ground_filter = Node(
        package="faucon_perception",
        executable="lidar_calibrator",
        name="lidar_calibrator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    
    rviz = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', use_rviz, '" == "true"'])),
    )

    
    spawn_robot = TimerAction(
        period=15.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "/robot_description",
                "-name", "bot",
                "-world", "virtual_maize_field",
                "-allow_renaming", "false",
                "-x", "-3.81",  #-2.28
                "-y", spawn_y,
                "-z", spawn_z,
                "-R", "-0.01",
                "-P", "-0.03",
                "-Y", "1.52",
            ],
        )]
    )

   
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_path, "config", "bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
            "/camera/depth_image",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "20"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_controller"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_params,
            {
                "use_sim_time": use_sim_time, 
                "use_stamped": PythonExpression(['"', use_ros2_control_value, '" == "true"'])
            },
        ],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

   
    return [
        environment,
        launch_world,
        robot_state_publisher,
        joint_state_publisher_node,
        rviz,
        spawn_robot,
        gz_bridge,
        control_4ws,
        ros_gz_image_bridge,
        steer_controller,
        velocity_controller,
        joint_broad_spawner,
        # twist_mux,
        #gz_sim,
        ground_filter, 
        faucon_ihm, 
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value="true", 
                description="Use sim time if true"
            ),
            DeclareLaunchArgument(
                "use_mini", 
                default_value="false", 
                description="Use mini robot if true (forces use_ros2_control=false)"
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="ROS2 control enabled if true (ignored if use_mini=true)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                 default_value="false",
                 description="Launch RViz if true (ignored if use_mini=true)",
            ),
            DeclareLaunchArgument(
                "headless",
                 default_value="false",
                 description="Start Gazebo server without GUI rendering",
            ),
            DeclareLaunchArgument(
                "verbose",
                 default_value="false",
                 description="Increase Gazebo verbosity",
            ),
        
            OpaqueFunction(function=launch_setup),
        ]
    )
