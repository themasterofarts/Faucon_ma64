from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
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
    headless = LaunchConfiguration("headless").perform(context).lower()
    use_lidar_calibrator = LaunchConfiguration("use_lidar_calibrator").perform(context).lower()
    
    
    if use_mini_value.lower() == "true":
        robot_desc = xacro.process_file(xacro_file_mini).toxml()
        spawn_z = "0.4"
        spawn_y = "-2.94"
        use_ros2_control_value = "false"
    else:
        use_ros2_control_value = LaunchConfiguration("use_ros2_control").perform(context).lower()
        # L'argument doit etre transmis au xacro : il choisit ros2_control ou le
        # DiffDrive Gazebo, et le type (revolute / fixed) des jambes de braquage.
        robot_desc = xacro.process_file(
            xacro_file, mappings={"use_ros2_control": use_ros2_control_value}
        ).toxml()
        spawn_z = "1.0"
        spawn_y = "-2.83"

    # Conditions calculees une seule fois (auparavant : PythonExpression repete 6 fois)
    #  - with_ros2_control : robot complet 4WS (controleurs ros2_control + control_4ws)
    #  - without_ros2_control : DiffDrive Gazebo (robot mini, ou use_ros2_control:=false)
    with_ros2_control = IfCondition(use_ros2_control_value)
    without_ros2_control = UnlessCondition(use_ros2_control_value)

    twist_mux_params = os.path.join(pkg_path, "config", "twist_mux.yaml")

   
    environment = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        path.join(pkg_path, "worlds"),
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
            "world_name": "generated.world",
            # true (defaut) : Gazebo sans interface graphique (~2 coeurs et ~0,9 Go
            # economises) ; les capteurs (camera, LiDAR) restent simules.
            "headless": headless,
        }.items(),
    )

    faucon_ihm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_faucon_ihm, "launch", "ihm_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # false (defaut) : IHM / rosbridge accessibles en local uniquement
            "ihm_lan": LaunchConfiguration("ihm_lan"),
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

    
    # Uniquement sans ros2_control (robot mini, ou use_ros2_control:=false) :
    # avec ros2_control, le joint_state_broadcaster publie deja les vrais etats
    # sur /joint_states, et joint_state_publisher (qui publie des zeros) entrait
    # en conflit avec lui (TF des roues / jambes qui sautaient).
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
        condition=without_ros2_control,
    )

   
    control_4ws = Node(
        package="faucon_control",
        executable="control_4ws.py",
        name="control_4ws",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=with_ros2_control,
    )

    # Modes DiffDrive (mini, use_ros2_control:=false) : control_4ws n'est pas lance,
    # ce noeud assure l'arret sur /e_stop et le watchdog /cmd_vel.
    diffdrive_safety = Node(
        package="faucon_control",
        executable="diffdrive_safety.py",
        name="diffdrive_safety",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=without_ros2_control,
    )

    # Retrait du sol du nuage LiDAR (/cloud -> /cloud_calib_out, /ground_cloud).
    # Necessaire a la cartographie RTAB-Map et a la detection de rangs : desactive
    # par defaut (~0,6 coeur), activer avec use_lidar_calibrator:=true.
    ground_filter = Node(
        package="faucon_perception",
        executable="lidar_calibrator",
        name="lidar_calibrator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_lidar_calibrator),
    )

    
    rviz = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    
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
            spawn_y,
            "-z",
            spawn_z,
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
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    # /camera/image seulement : /camera/depth_image passe par le parameter_bridge
    # (bridge.yaml) en mode lazy, qu'image_bridge ne gere pas.
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "20"],
        condition=with_ros2_control,
    )

    steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_controller"],
        condition=with_ros2_control,
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        condition=with_ros2_control,
    )

    
    # Arbitrage des commandes de vitesse : Nav2 (/cmd_vel_nav2) et teleop
    # (/cmd_vel_teleop) -> /cmd_vel, lu par control_4ws.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {
                "use_sim_time": use_sim_time,
                # Nav2 Jazzy et l'IHM publient des Twist (non stamped) ;
                # twist_mux Jazzy est en TwistStamped par defaut.
                "use_stamped": False,
            },
        ],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

   
    return [
        environment,
        robot_state_publisher,
        joint_state_publisher_node,
        rviz,
        spawn_robot,
        gz_bridge,
        control_4ws,
        diffdrive_safety,
        launch_world,
        ros_gz_image_bridge,
        steer_controller,
        velocity_controller,
        joint_broad_spawner,
        twist_mux,
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
                description="true: ros2_control + control_4ws (4 roues directrices) ; "
                            "false: DiffDrive Gazebo, jambes de braquage fixes (ignored if use_mini=true)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                 default_value="false",
                 description="Launch RViz if true (ignored if use_mini=true)",
            ),
        
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="true : Gazebo sans interface graphique (capteurs toujours simules, "
                            "economise ~2 coeurs et ~0,9 Go) ; false : interface Gazebo affichee",
            ),
            DeclareLaunchArgument(
                "use_lidar_calibrator",
                default_value="false",
                description="true : lance lidar_calibrator (/cloud_calib_out), requis par la "
                            "cartographie RTAB-Map et la detection de rangs",
            ),
            DeclareLaunchArgument(
                "ihm_lan",
                default_value="false",
                description="true : IHM, rosbridge et camera accessibles depuis le reseau "
                            "(SANS authentification) ; false : machine locale uniquement",
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )