"""Launch the PX4 X500 drone inside a running Faucon Gazebo world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _workspace_root_from_share(package_share_dir):
    return os.path.abspath(os.path.join(package_share_dir, '../../../..'))


def _validate_px4_dir(context, *, px4_dir_config, workspace_root):
    px4_dir = os.path.abspath(os.path.expanduser(px4_dir_config.perform(context)))
    workspace_root = os.path.abspath(workspace_root)
    if os.path.commonpath([px4_dir, workspace_root]) != workspace_root:
        raise RuntimeError(
            f'px4_dir must stay inside the Faucon workspace: {workspace_root}. '
            f'Got: {px4_dir}'
        )

    required_paths = [
        os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4'),
        os.path.join(px4_dir, 'build', 'px4_sitl_default', 'etc'),
        os.path.join(
            px4_dir, 'Tools', 'simulation', 'gz', 'models',
            'x500_mono_cam', 'model.sdf'
        ),
    ]
    missing = [path for path in required_paths if not os.path.exists(path)]
    if missing:
        missing_text = ', '.join(missing)
        raise RuntimeError(
            'PX4-Autopilot must be available inside the Faucon workspace. '
            f'Expected px4_dir={px4_dir}. Missing: {missing_text}'
        )
    return []


def generate_launch_description():
    pkg_drone = get_package_share_directory('faucon_drone')
    workspace_root = _workspace_root_from_share(pkg_drone)
    default_px4_dir = os.path.join(workspace_root, 'PX4-Autopilot')
    camera_bridge_config = os.path.join(pkg_drone, 'config', 'x500_camera_bridge.yaml')

    px4_dir = LaunchConfiguration('px4_dir')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    px4_bin = PathJoinSubstitution([px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4'])
    px4_etc = PathJoinSubstitution([px4_dir, 'build', 'px4_sitl_default', 'etc'])
    x500_sdf = PathJoinSubstitution([
        px4_dir, 'Tools', 'simulation', 'gz', 'models', 'x500_mono_cam', 'model.sdf'
    ])

    # ── Spawn X500 SDF into the running UGV Gazebo world ─────────────────
    # ros_gz_sim create accepts a file path — no need for GZ_SIM_RESOURCE_PATH
    # to be set here (Gazebo itself needs it, handled by drone_sim.launch.py)
    spawn_x500 = Node(
        package='ros_gz_sim',
        executable='create',
        name='x500_spawner',
        output='screen',
        arguments=[
            '-world', world_name,
            '-file', x500_sdf,
            '-name', model_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
    )

    # ── XRCE-DDS agent — bridges PX4 uORB ↔ ROS 2 ───────────────────────
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='xrce_dds_agent',
        output='screen',
    )

    # ── PX4 SITL — standalone mode (connects to running Gazebo) ──────────
    # PX4_GZ_STANDALONE=1  : do not launch Gazebo, connect to running instance
    # PX4_GZ_MODEL_NAME    : name of the model already spawned in Gazebo
    # PX4_SIM_MODEL        : selects the PX4 vehicle config (params, mixer)
    px4_sitl = ExecuteProcess(
        cmd=[px4_bin, px4_etc, '-s', 'etc/init.d-posix/rcS'],
        name='px4_sitl',
        cwd=px4_dir,
        output='screen',
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_MODEL_NAME': model_name,
            'PX4_GZ_WORLD': world_name,
            'PX4_SIM_MODEL': 'gz_x500_mono_cam',
        },
    )

    # ── PX4X500Adapter — /fmu/* ↔ /faucon/drone/* ────────────────────────
    adapter = Node(
        package='faucon_drone',
        executable='px4_x500_adapter',
        name='px4_x500_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_timeout': 0.5,
            'max_velocity_xy': 2.0,
            'max_velocity_z': 1.0,
            'max_yaw_rate': 1.0,
            'force_arm': False,
        }],
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='x500_camera_bridge',
        output='screen',
        parameters=[{
            'config_file': camera_bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir', default_value=default_px4_dir,
            description='PX4-Autopilot path, constrained to the Faucon workspace'),
        DeclareLaunchArgument(
            'world_name', default_value='virtual_maize_field',
            description='Gazebo world name'),
        DeclareLaunchArgument(
            'model_name', default_value='x500_mono_cam_0',
            description='Gazebo model name for the X500'),
        DeclareLaunchArgument(
            'spawn_x', default_value='2.0',
            description='X spawn position (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Y spawn position (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.3',
            description='Z spawn position (m, above ground)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        OpaqueFunction(
            function=_validate_px4_dir,
            kwargs={'px4_dir_config': px4_dir, 'workspace_root': workspace_root},
        ),
        # t=0s : spawn model + start agent
        xrce_agent,
        spawn_x500,
        # t=4s : PX4 SITL (needs Gazebo + model to be ready)
        TimerAction(period=4.0,  actions=[px4_sitl]),
        # t=10s: adapter + camera bridge (needs XRCE-DDS + PX4 to be up)
        TimerAction(period=10.0, actions=[adapter, camera_bridge]),
    ])
