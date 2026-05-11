"""
launch/ihm_launch.py
────────────────────
Lance en une seule commande :
  1. port_cleanup          (libère 9090/8080/3000 si occupés — transition propre devcontainer/local)
  2. rosbridge_websocket   (port 9090)
  3. web_video_server      (port 8080)
  4. ihm_server            (serveur HTTP Python pour l'IHM React, port 3000)

Usage :
  ros2 launch faucon_ihm ihm_launch.py
  ros2 launch faucon_ihm ihm_launch.py rosbridge_port:=9090 video_port:=8080 ihm_port:=3000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port", default_value="9090",
        description="Port WebSocket de rosbridge"
    )
    video_port_arg = DeclareLaunchArgument(
        "video_port", default_value="8080",
        description="Port HTTP de web_video_server"
    )
    ihm_port_arg = DeclareLaunchArgument(
        "ihm_port", default_value="3000",
        description="Port HTTP du serveur IHM"
    )
    ihm_host_arg = DeclareLaunchArgument(
        "ihm_host", default_value="0.0.0.0",
        description="Interface d'écoute du serveur IHM (0.0.0.0 = tout le LAN)"
    )

    rosbridge_port = LaunchConfiguration("rosbridge_port")
    video_port     = LaunchConfiguration("video_port")
    ihm_port       = LaunchConfiguration("ihm_port")
    ihm_host       = LaunchConfiguration("ihm_host")

    # ── Libère les ports avant de démarrer (transition devcontainer/local propre) ──
    port_cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "fuser -k 9090/tcp 8080/tcp 3000/tcp 2>/dev/null; sleep 0.5; true"],
        name="port_cleanup",
        output="screen",
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": rosbridge_port,
            "address": "",
            "retry_startup_delay": 5.0,
            "fragment_timeout": 600,
            "delay_between_messages": 0.0,
            "max_message_size": 10000000,
            "unregister_timeout": 10.0,
            "use_compression": False,
        }],
    )

    video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[{
            "port": video_port,
            "address": "0.0.0.0",
            "server_threads": 2,
            "ros_threads": 2,
        }],
    )

    ihm_dir = PathJoinSubstitution([
        FindPackageShare("faucon_ihm"), "ihm"
    ])

    ihm_server = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("faucon_ihm"), "launch", "ihm_server.py"]),
            ihm_host,
            ihm_port,
            ihm_dir,
        ],
        name="ihm_http_server",
        output="screen",
    )

    log_start = LogInfo(msg=[
        "\n",
        "╔══════════════════════════════════════════════╗\n",
        "║        ROS2 IHM — GROUND CONTROL             ║\n",
        "╠══════════════════════════════════════════════╣\n",
        "║  rosbridge   ws://0.0.0.0:", rosbridge_port, "          ║\n",
        "║  camera      http://0.0.0.0:", video_port, "        ║\n",
        "║  IHM         http://0.0.0.0:", ihm_port, "         ║\n",
        "╚══════════════════════════════════════════════╝\n",
    ])

    # Démarre les nœuds uniquement après que port_cleanup a terminé
    start_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=port_cleanup,
            on_exit=[log_start, rosbridge_node, video_server_node, ihm_server],
        )
    )

    return LaunchDescription([
        rosbridge_port_arg,
        video_port_arg,
        ihm_port_arg,
        ihm_host_arg,
        port_cleanup,
        start_nodes,
    ])
