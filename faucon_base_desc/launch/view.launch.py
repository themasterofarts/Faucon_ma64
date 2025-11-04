from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from os import path


def generate_launch_description():
    pkg_path = get_package_share_directory("faucon_base_desc")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_maize_field = get_package_share_directory("virtual_maize_field")
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()

    twist_mux_params = os.path.join(
        get_package_share_directory("faucon_base_desc"), "config", "twrist_mux.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

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

    environment = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        path.join(get_package_share_directory("faucon_base_desc"), "worlds"),
    )

    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_maize_field, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world_path": os.path.join(pkg_path, "worlds", "virtual_maize_field"),
            "world_name": "generated.world",
        }.items(),
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

    control_4ws = Node(
        package="faucon_control",
        executable="control_4ws.py",
        name="control_4ws",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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
            "-2.28",
            "-y",
            "-3.83",
            "-z",
            "0.7",
            "-R",
            "-0.01",
            "-P",
            "-0.03",
            "-Y",
            "1.52",
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

    # Spawn ROS2 Control controllers



    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "20"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_controller"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time, "use_stamped": use_ros2_control},
        ],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time if true"
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="ROS2 control enabled if true",
            ),
            robot_state_publisher,
            # joint_state_publisher_gui,
            joint_state_publisher_node,
            #rviz,
            spawn_robot,
            gz_bridge,
            #gz_sim,
            control_4ws,
            environment,
            launch_world,
            ros_gz_image_bridge,
            steer_controller,
            velocity_controller,
            # diff_drive_spawner,
            joint_broad_spawner,
            #twist_mux,
        ]
    )
