"""Launch the full Faucon UGV and PX4 X500 drone co-simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _workspace_root_from_share(package_share_dir):
    return os.path.abspath(os.path.join(package_share_dir, '../../../..'))


def _validate_px4_dir(context, *, px4_dir_config, workspace_root):
    candidate = os.path.abspath(os.path.expanduser(px4_dir_config.perform(context)))
    workspace_root = os.path.abspath(workspace_root)
    if os.path.commonpath([candidate, workspace_root]) != workspace_root:
        raise RuntimeError(
            f'px4_dir must stay inside the Faucon workspace: {workspace_root}. '
            f'Got: {candidate}'
        )

    required_paths = [
        os.path.join(candidate, 'build', 'px4_sitl_default', 'bin', 'px4'),
        os.path.join(candidate, 'build', 'px4_sitl_default', 'etc'),
        os.path.join(
            candidate, 'Tools', 'simulation', 'gz', 'models',
            'x500_mono_cam', 'model.sdf'
        ),
    ]
    missing = [path for path in required_paths if not os.path.exists(path)]
    if missing:
        missing_text = ', '.join(missing)
        raise RuntimeError(
            'PX4-Autopilot is required and must be built inside the Faucon workspace '
            'before Gazebo starts. '
            f'Expected px4_dir={candidate}. Missing: {missing_text}.'
        )
    return []


def generate_launch_description():
    pkg_base_desc = get_package_share_directory('faucon_base_desc')
    pkg_drone = get_package_share_directory('faucon_drone')
    workspace_root = _workspace_root_from_share(pkg_drone)
    default_px4_dir = os.path.join(workspace_root, 'PX4-Autopilot')

    px4_dir              = LaunchConfiguration('px4_dir')
    use_sim_time         = LaunchConfiguration('use_sim_time')
    use_mini             = LaunchConfiguration('use_mini')
    use_rviz             = LaunchConfiguration('use_rviz')
    headless             = LaunchConfiguration('headless')
    verbose              = LaunchConfiguration('verbose')
    gazebo_world_name    = LaunchConfiguration('gazebo_world_name')
    drone_spawn_x        = LaunchConfiguration('drone_spawn_x')
    drone_spawn_y        = LaunchConfiguration('drone_spawn_y')
    drone_spawn_z        = LaunchConfiguration('drone_spawn_z')
    drone_spawn_yaw      = LaunchConfiguration('drone_spawn_yaw')
    auto_takeoff         = LaunchConfiguration('auto_takeoff')
    takeoff_altitude     = LaunchConfiguration('takeoff_altitude')
    auto_takeoff_delay   = LaunchConfiguration('auto_takeoff_delay')
    force_arm            = LaunchConfiguration('force_arm')
    auto_trajectory      = LaunchConfiguration('auto_trajectory')
    trajectory_waypoints = LaunchConfiguration('trajectory_waypoints')

    # ── Extend GZ resource path BEFORE Gazebo starts ─────────────────────
    # Gazebo must resolve x500_base and x500_mono_cam models from PX4 tools.
    gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'models']),
            ':',
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'worlds']),
            ':',
            PathJoinSubstitution([pkg_drone, 'models']),
        ],
    )

    # ── UGV simulation ────────────────────────────────────────────────────
    ugv_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_base_desc, 'launch', 'view.launch.py')
        ),
        launch_arguments={
            'use_sim_time':      use_sim_time,
            'use_mini':          use_mini,
            'use_rviz':          use_rviz,
            'headless':          headless,
            'verbose':           verbose,
            'use_ros2_control':  'true',
        }.items(),
    )

    # ── Drone spawn (t=20s — Gazebo + UGV must be fully loaded) ──────────
    # view.launch.py spawns the UGV at t=15s; wait 5s beyond that so the UGV
    # ros2_control stack is active before the drone enters the world.
    drone_spawn = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drone, 'launch', 'spawn_drone_px4.launch.py')
                ),
                launch_arguments={
                    'px4_dir':             px4_dir,
                    'world_name':          gazebo_world_name,
                    'use_sim_time':        use_sim_time,
                    'spawn_x':             drone_spawn_x,
                    'spawn_y':             drone_spawn_y,
                    'spawn_z':             drone_spawn_z,
                    'spawn_yaw':           drone_spawn_yaw,
                    'auto_takeoff':        auto_takeoff,
                    'takeoff_altitude':    takeoff_altitude,
                    'auto_takeoff_delay':  auto_takeoff_delay,
                    'force_arm':           force_arm,
                    'auto_trajectory':     auto_trajectory,
                    'trajectory_waypoints': trajectory_waypoints,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir', default_value=default_px4_dir,
            description='PX4-Autopilot path, constrained to the Faucon workspace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'use_mini', default_value='false',
            description='Use mini UGV model'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Launch Gazebo server without GUI rendering'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Increase Gazebo verbosity'),
        DeclareLaunchArgument(
            'gazebo_world_name', default_value='virtual_maize_field',
            description='Gazebo internal world name (keep different from the .world filename)'),
        DeclareLaunchArgument(
            'drone_spawn_x', default_value='5.5',
            description='Drone spawn X (m, world frame) — outside maize rows by default'),
        DeclareLaunchArgument(
            'drone_spawn_y', default_value='0.0',
            description='Drone spawn Y (m, world frame)'),
        DeclareLaunchArgument(
            'drone_spawn_z', default_value='1.5',
            description='Drone spawn Z (m, above ground)'),
        DeclareLaunchArgument(
            'drone_spawn_yaw', default_value='0.0',
            description='Drone spawn yaw (rad)'),
        DeclareLaunchArgument(
            'auto_takeoff', default_value='true',
            description='Automatically arm, take off, and hold hover '
                        '(ignored when auto_trajectory:=true)'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='3.0',
            description='Auto takeoff target altitude (m)'),
        DeclareLaunchArgument(
            'auto_takeoff_delay', default_value='35.0',
            description='Delay before auto takeoff, to let PX4 sensors/EKF settle (s)'),
        DeclareLaunchArgument(
            'force_arm', default_value='true',
            description='Force arm in autonomous SITL if health checks complain'),
        DeclareLaunchArgument(
            'auto_trajectory', default_value='false',
            description='Launch drone_trajectory instead of drone_takeoff_hover. '
                        'Handles arm + takeoff + waypoint sequence autonomously.'),
        DeclareLaunchArgument(
            'trajectory_waypoints', default_value='',
            description='Comma-separated flat waypoint list "x0,y0,z0,x1,y1,z1,…" (world-ENU, m). '
                        'Empty = take off and hover only.'),

        OpaqueFunction(
            function=_validate_px4_dir,
            kwargs={'px4_dir_config': px4_dir, 'workspace_root': workspace_root},
        ),
        gz_resources,
        ugv_sim,
        drone_spawn,
    ])
