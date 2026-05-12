"""Launch the PX4 X500 drone inside a running Faucon Gazebo world."""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

_RESOLVED_SDF_PATH = '/tmp/faucon_x500_spawn.sdf'


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


def _sync_px4_airframe(context, *, px4_dir_config, source_airframe, source_post):
    px4_dir = os.path.abspath(os.path.expanduser(px4_dir_config.perform(context)))
    target_dir = os.path.join(
        px4_dir, 'build', 'px4_sitl_default', 'etc', 'init.d-posix',
        'airframes'
    )
    os.makedirs(target_dir, exist_ok=True)
    shutil.copyfile(
        source_airframe,
        os.path.join(target_dir, '4010_gz_x500_mono_cam')
    )
    target_post = os.path.join(target_dir, '4010_gz_x500_mono_cam.post')
    shutil.copyfile(source_post, target_post)
    return []


def _resolve_x500_sdf(context, *, px4_dir_config, pkg_drone):
    """Pre-resolve the x500_mono_cam_faucon SDF using gz sdf -p + SDF_PATH.

    ros_gz_sim create -file sends the SDF string to Gazebo, which then resolves
    <include merge="true"> internally. In Gazebo Harmonic 8.x, nested merge-includes
    spawned dynamically fail to register visual components in the rendering scene —
    physics and sensors work but the model is invisible in the GUI.

    gz sdf -p with SDF_PATH set resolves all nested includes at launch time and
    produces a fully flat SDF. Spawning that flat SDF via -file avoids the issue.
    """
    px4_dir = os.path.abspath(os.path.expanduser(px4_dir_config.perform(context)))
    model_file = os.path.join(pkg_drone, 'models', 'x500_mono_cam_faucon', 'model.sdf')

    sdf_path = os.pathsep.join([
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(pkg_drone, 'models'),
    ])

    env = {**os.environ, 'SDF_PATH': sdf_path}
    result = subprocess.run(
        ['gz', 'sdf', '-p', model_file],
        env=env, capture_output=True, text=True
    )

    if result.returncode != 0 or '<link' not in result.stdout:
        raise RuntimeError(
            f'Failed to resolve x500_mono_cam_faucon SDF.\n'
            f'SDF_PATH={sdf_path}\n'
            f'stderr: {result.stderr}'
        )

    with open(_RESOLVED_SDF_PATH, 'w') as f:
        f.write(result.stdout)

    return []


def generate_launch_description():
    pkg_drone = get_package_share_directory('faucon_drone')
    workspace_root = _workspace_root_from_share(pkg_drone)
    default_px4_dir = os.path.join(workspace_root, 'PX4-Autopilot')
    camera_bridge_config = os.path.join(pkg_drone, 'config', 'x500_camera_bridge.yaml')
    airframe_post = os.path.join(
        pkg_drone, 'config', 'px4_params', '4010_gz_x500_mono_cam.post'
    )
    airframe = os.path.join(
        pkg_drone, 'config', 'px4_params', '4010_gz_x500_mono_cam'
    )

    px4_dir = LaunchConfiguration('px4_dir')
    world_name = LaunchConfiguration('world_name')
    gazebo_world_name = PythonExpression([
        "'virtual_maize_field' if '", world_name,
        "'.endswith('.world') or '", world_name,
        "'.endswith('.sdf') else '", world_name, "'"
    ])
    model_name = LaunchConfiguration('model_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_takeoff = LaunchConfiguration('auto_takeoff')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    auto_takeoff_delay = LaunchConfiguration('auto_takeoff_delay')
    force_arm = LaunchConfiguration('force_arm')
    auto_trajectory = LaunchConfiguration('auto_trajectory')
    trajectory_waypoints = LaunchConfiguration('trajectory_waypoints')
    # auto_takeoff_hover runs only when auto_takeoff=true AND auto_trajectory is not active
    _takeoff_only = PythonExpression([
        '"', auto_takeoff, '" == "true" and "', auto_trajectory, '" != "true"'
    ])
    px4_bin = PathJoinSubstitution([px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4'])
    px4_etc = PathJoinSubstitution([px4_dir, 'build', 'px4_sitl_default', 'etc'])

    # ── Spawn X500 SDF into the running UGV Gazebo world ─────────────────
    # Uses a pre-resolved flat SDF (written by _resolve_x500_sdf above).
    # The flat SDF has all nested includes inlined, so Gazebo's rendering
    # scene receives the visual components correctly on Harmonic 8.x.
    spawn_x500 = Node(
        package='ros_gz_sim',
        executable='create',
        name='x500_spawner',
        output='screen',
        arguments=[
            '-world', gazebo_world_name,
            '-file', _RESOLVED_SDF_PATH,
            '-name', model_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
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
    # GZ_IP                : match PX4's standard gz target and force local discovery
    px4_sitl = ExecuteProcess(
        cmd=[px4_bin, px4_etc, '-s', 'etc/init.d-posix/rcS'],
        name='px4_sitl',
        cwd=px4_dir,
        output='screen',
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_MODEL_NAME': model_name,
            'PX4_GZ_WORLD': gazebo_world_name,
            'PX4_SIM_MODEL': 'gz_x500_mono_cam',
            'GZ_IP': '127.0.0.1',
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
            'force_arm': force_arm,
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

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='x500_ground_truth_odom_bridge',
        output='screen',
        arguments=[
            ['/model/', model_name, '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        ],
        remappings=[
            ((['/model/', model_name, '/odometry']), '/faucon/drone/gz_odom'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='x500_ground_truth_pose_bridge',
        output='screen',
        arguments=[
            ['/model/', model_name, '/pose@geometry_msgs/msg/Pose[gz.msgs.Pose'],
        ],
        remappings=[
            ((['/model/', model_name, '/pose']), '/faucon/drone/gz_pose'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    auto_takeoff_hover = Node(
        package='faucon_drone',
        executable='drone_takeoff_hover',
        name='drone_takeoff_hover',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_altitude': takeoff_altitude,
            'start_delay': auto_takeoff_delay,
            'prearm_setpoint_time': 2.0,
            'takeoff_speed': 0.8,
            'hover_kp_xy': 0.6,
            'hover_kp_z': 0.7,
            'max_velocity_xy': 0.8,
            'max_velocity_z': 0.8,
            'altitude_tolerance': 0.15,
        }],
        condition=IfCondition(_takeoff_only),
    )

    # ── Waypoint trajectory node (mutually exclusive with drone_takeoff_hover) ──
    # Handles its own arm + takeoff + trajectory + hover.
    # Waypoints via launch: trajectory_waypoints:="x0,y0,z0,x1,y1,z1,…"
    # Waypoints via CLI:    --ros-args -p waypoints:=[x0,y0,z0,x1,y1,z1,…]
    drone_trajectory = Node(
        package='faucon_drone',
        executable='drone_trajectory',
        name='drone_trajectory',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_str': trajectory_waypoints,
            'takeoff_altitude': takeoff_altitude,
            'start_delay': auto_takeoff_delay,
            'prearm_setpoint_time': 2.0,
            'takeoff_speed': 0.8,
            'cruise_speed': 1.2,
            'hover_kp_xy': 0.6,
            'hover_kp_z': 0.7,
            'max_velocity_xy': 1.5,
            'max_velocity_z': 0.8,
            'waypoint_tolerance': 0.5,
            'altitude_tolerance': 0.15,
            'return_home': False,
        }],
        condition=IfCondition(auto_trajectory),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir', default_value=default_px4_dir,
            description='PX4-Autopilot path, constrained to the Faucon workspace'),
        DeclareLaunchArgument(
            'world_name', default_value='virtual_maize_field',
            description='Gazebo internal world name, not the .world file name'),
        DeclareLaunchArgument(
            'model_name', default_value='x500_mono_cam_0',
            description='Gazebo model name for the X500'),
        DeclareLaunchArgument(
            'spawn_x', default_value='5.5',
            description='X spawn position (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Y spawn position (m, world frame)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='1.5',
            description='Z spawn position (m, above ground)'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='X500 spawn yaw (rad)'),
        DeclareLaunchArgument(
            'auto_takeoff', default_value='true',
            description='Automatically arm, take off, and hold hover'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='3.0',
            description='Auto takeoff target altitude in the local ENU frame (m)'),
        DeclareLaunchArgument(
            'auto_takeoff_delay', default_value='35.0',
            description='Delay before auto takeoff, to let PX4 sensors/EKF settle (s)'),
        DeclareLaunchArgument(
            'force_arm', default_value='true',
            description='Use PX4 force-arm code for autonomous SITL health-check bypass'),
        DeclareLaunchArgument(
            'auto_trajectory', default_value='false',
            description='Launch drone_trajectory instead of drone_takeoff_hover. '
                        'Handles arm + takeoff + waypoint sequence autonomously.'),
        DeclareLaunchArgument(
            'trajectory_waypoints', default_value='',
            description='Comma-separated flat waypoint list "x0,y0,z0,x1,y1,z1,…" (world-ENU, m). '
                        'Empty = take off and hover only.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        OpaqueFunction(
            function=_validate_px4_dir,
            kwargs={'px4_dir_config': px4_dir, 'workspace_root': workspace_root},
        ),
        OpaqueFunction(
            function=_sync_px4_airframe,
            kwargs={
                'px4_dir_config': px4_dir,
                'source_airframe': airframe,
                'source_post': airframe_post,
            },
        ),
        OpaqueFunction(
            function=_resolve_x500_sdf,
            kwargs={'px4_dir_config': px4_dir, 'pkg_drone': pkg_drone},
        ),
        # t=0s : spawn model + start agent
        xrce_agent,
        spawn_x500,
        # t=6s : PX4 SITL (needs Gazebo + model to be ready)
        TimerAction(period=6.0,  actions=[px4_sitl]),
        # t=12s: adapter + camera bridge (needs XRCE-DDS + PX4 to be up)
        TimerAction(period=12.0, actions=[
            adapter,
            camera_bridge,
            odom_bridge,
            pose_bridge,
            auto_takeoff_hover,
            drone_trajectory,
        ]),
    ])
