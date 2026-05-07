"""
Cartographie 3D du champ de maïs avec RTAB-Map lidar seul.

Prérequis (à lancer avant) : ./faucon/launch_full.sh (description et navigation)

Topics publiés par RTAB-Map :
  /rtabmap/cloud_map      
  /rtabmap/map            
  /rtabmap/mapData   
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    use_sim_time  = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    open_viz      = LaunchConfiguration('open_viz')
    fresh_start   = LaunchConfiguration('fresh_start').perform(context)

    rtabmap_params = PathJoinSubstitution([
        FindPackageShare('faucon_perception'), 'config', 'rtabmap.yaml'
    ])

    # --delete_db_on_start efface la DB au démarrage → carte fraîche à chaque session
    rtabmap_args = ['--delete_db_on_start'] if fresh_start.lower() == 'true' else []

    # RTAB-Map SLAM 
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=rtabmap_args,
        parameters=[
            rtabmap_params,
            {
                'use_sim_time':         use_sim_time,
                'database_path':        database_path,
                'frame_id':             'base_link',
                'odom_frame_id':        'odom',
                'map_frame_id':         'map',
                'publish_tf':           False,
                'subscribe_depth':      False,
                'subscribe_rgb':        False,
                'subscribe_scan_cloud': True,
                'subscribe_rgbd':       False,
                'subscribe_stereo':     False,
                'approx_sync':          True,
            }
        ],
        remappings=[
            # lidar_calibrator publie /cloud_calib_out (sol retiré par RANSAC)
            ('scan_cloud', '/cloud_calib_out'),
            ('odom',       '/odometry/local'),
        ],
    )

    # Visualiseur RTAB-Map 
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan_cloud', '/cloud_calib_out'),
            ('odom',       '/odometry/local'),
        ],
        condition=IfCondition(open_viz),
    )

    return [rtabmap_node, rtabmap_viz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Utiliser le temps simulé Gazebo'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/faucon_map2.db',
            description='Chemin de la base de données RTAB-Map'
        ),
        DeclareLaunchArgument(
            'open_viz',
            default_value='false',
            description='Ouvrir le visualiseur RTAB-Map (true/false)'
        ),
        DeclareLaunchArgument(
            'fresh_start',
            default_value='true',
            description='true = efface la carte précédente, false = reprend la session'
        ),
        OpaqueFunction(function=launch_setup),
    ])
