"""Launch the full Faucon UGV and drone co-simulation."""

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression


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
            'PX4-Autopilot is required for controller:=px4, and must be built '
            'inside the Faucon workspace before Gazebo starts. '
            f'Expected px4_dir={candidate}. Missing: {missing_text}. '
            'Use controller:=sim to run the lightweight fallback without PX4.'
        )
    return []


def generate_launch_description():
    pkg_base_desc = get_package_share_directory('faucon_base_desc')
    pkg_drone = get_package_share_directory('faucon_drone')
    workspace_root = _workspace_root_from_share(pkg_drone)
    default_px4_dir = os.path.join(workspace_root, 'PX4-Autopilot')

    controller = LaunchConfiguration('controller')
    px4_dir = LaunchConfiguration('px4_dir')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mini = LaunchConfiguration('use_mini')
    use_rviz = LaunchConfiguration('use_rviz')
    world_name = LaunchConfiguration('world_name')
    drone_spawn_x = LaunchConfiguration('drone_spawn_x')
    drone_spawn_y = LaunchConfiguration('drone_spawn_y')
    drone_spawn_z = LaunchConfiguration('drone_spawn_z')
    drone_spawn_yaw = LaunchConfiguration('drone_spawn_yaw')
    is_px4 = PythonExpression(['"', controller, '" == "px4"'])
    is_sim = PythonExpression(['"', controller, '" == "sim"'])

    # ── Extend GZ resource path BEFORE Gazebo starts ─────────────────────
    # Gazebo must be able to resolve x500_base and x500_mono_cam models.
    # AppendEnvironmentVariable modifies the environment for all child processes
    # launched by this launch description, including gz_sim inside view.launch.py.
    gz_resources_px4 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'models']),
            ':',
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'worlds']),
        ],
        condition=IfCondition(is_px4),
    )

    # ── UGV simulation ────────────────────────────────────────────────────
    ugv_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_base_desc, 'launch', 'view.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_mini': use_mini,
            'use_rviz': use_rviz,
            'use_ros2_control': 'true',
        }.items(),
    )

    # ── Drone spawn (t=12s — Gazebo + UGV must be fully loaded) ──────────
    drone_spawn = TimerAction(
        period=12.0,
        actions=[
            # PX4 path: X500 + SITL + XRCE-DDS + adapter
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drone, 'launch', 'spawn_drone_px4.launch.py')
                ),
                launch_arguments={
                    'px4_dir': px4_dir,
                    'world_name': world_name,
                    'use_sim_time': use_sim_time,
                    'spawn_x': drone_spawn_x,
                    'spawn_y': drone_spawn_y,
                    'spawn_z': drone_spawn_z,
                }.items(),
                condition=IfCondition(is_px4),
            ),
            # Sim fallback: custom URDF drone + velocity controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drone, 'launch', 'spawn_drone.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'spawn_x': drone_spawn_x,
                    'spawn_y': drone_spawn_y,
                    'spawn_z': drone_spawn_z,
                    'spawn_yaw': drone_spawn_yaw,
                }.items(),
                condition=IfCondition(is_sim),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='px4',
            description='Drone controller: px4 (PX4 SITL + X500) or sim (velocity controller)'),
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
            'world_name', default_value='virtual_maize_field',
            description='Gazebo world name inside generated.world'),
        DeclareLaunchArgument(
            'drone_spawn_x', default_value='2.0',
            description='Drone spawn X (m, world frame) — offset from UGV'),
        DeclareLaunchArgument(
            'drone_spawn_y', default_value='0.0',
            description='Drone spawn Y (m, world frame)'),
        DeclareLaunchArgument(
            'drone_spawn_z', default_value='0.3',
            description='Drone spawn Z (m, above ground)'),
        DeclareLaunchArgument(
            'drone_spawn_yaw', default_value='0.0',
            description='Drone spawn yaw (rad, sim controller only)'),

        OpaqueFunction(
            function=_validate_px4_dir,
            kwargs={'px4_dir_config': px4_dir, 'workspace_root': workspace_root},
            condition=IfCondition(is_px4),
        ),
        gz_resources_px4,
        ugv_sim,
        drone_spawn,
    ])
