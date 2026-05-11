"""Launch the lightweight Faucon drone simulation fallback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_drone = get_package_share_directory('faucon_drone')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Process xacro at launch time
    drone_urdf = xacro.process_file(
        os.path.join(pkg_drone, 'description', 'drone.urdf.xacro')
    ).toxml()

    # ── robot_state_publisher (drone namespace avoids /robot_description clash) ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='drone',
        output='screen',
        parameters=[{
            'robot_description': drone_urdf,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn into Gazebo (reads description from /drone/robot_description) ──
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        name='drone_spawner',
        output='screen',
        arguments=[
            '-topic', '/drone/robot_description',
            '-name', 'drone',
            '-allow_renaming', 'false',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    # ── ros_gz_bridge (all drone topics except camera image) ──────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drone_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_drone, 'config', 'drone_bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Image bridge (camera image + camera_info) ─────────────────────────
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='drone_image_bridge',
        output='screen',
        arguments=['/drone/camera/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/drone/camera/image', '/faucon/drone/camera/image'),
            ('/drone/camera/camera_info', '/faucon/drone/camera/camera_info'),
        ],
    )

    # ── Velocity controller (cmd_vel → motor speed mixer) ─────────────────
    # Replaces gz-sim-multicopter-control-system which does not initialize
    # its publisher when a model is spawned dynamically into gz-sim8.
    velocity_controller = Node(
        package='faucon_drone',
        executable='drone_velocity_controller',
        name='drone_velocity_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'kz':   200.0,
            'kxy':   60.0,
            'kyaw':  40.0,
            'cmd_timeout': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Drone spawn X (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Drone spawn Y (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='5.0',
            description='Drone spawn Z/altitude (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Drone spawn yaw (rad)'),

        robot_state_publisher,
        gz_bridge,
        gz_image_bridge,
        velocity_controller,
        # Delay spawn by 2 s to let robot_state_publisher publish the description
        TimerAction(period=2.0, actions=[spawn_drone]),
    ])
