"""
Cartographie 3D RGBD du champ de maïs avec RTAB-Map lidar et camera.

Prérequis (à lancer avant) : ./faucon/launch_full.sh (description et navigation)

Topics consommés :
  /depth_camera/image        → RGB image 
  /depth_camera/depth_image  → image de profondeur float32 
  /depth_camera/camera_info  → intrinsèques
  /odometry/local            → odométrie EKF 

Topics publiés par RTAB-Map :
  /rtabmap/cloud_map   → PointCloud2 coloré (nuage 3D RGB)
  /rtabmap/map         → OccupancyGrid (grille 2D)
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
        FindPackageShare('faucon_perception'), 'config', 'rtabmap_rgbd.yaml'
    ])

    rtabmap_args = ['--delete_db_on_start'] if fresh_start.lower() == 'true' else []

    # RTAB-Map RGBD SLAM 
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=rtabmap_args,
        parameters=[
            rtabmap_params,
            {
                'use_sim_time':  use_sim_time,
                'database_path': database_path,
            }
        ],
        remappings=[
            ('rgb/image',       '/camera/image'),
            ('depth/image',     '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom',            '/odometry/local'),
            ('scan_cloud',      '/cloud_calib_out'),
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
            ('rgb/image',       '/camera/image'),
            ('depth/image',     '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom',            '/odometry/local'),
            ('scan_cloud',      '/cloud_calib_out'),
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
            default_value='~/.ros/faucon_map_rgbd.db',
            description='Chemin de la base de données RTAB-Map RGBD'
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
