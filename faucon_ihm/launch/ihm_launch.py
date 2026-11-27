"""
launch/ihm_launch.py
────────────────────
Lance en une seule commande :
  1. rosbridge_websocket  (port 9090)
  2. web_video_server     (port 8080)
  3. ihm_server           (serveur HTTP Python pour l'IHM React, port 3000)

Sécurité réseau :
  Par défaut, les trois serveurs n'écoutent QUE sur la machine locale (127.0.0.1).
  rosbridge n'a pas d'authentification : l'ouvrir au réseau permet à n'importe quel
  appareil du réseau de piloter le robot (/cmd_vel_teleop, /e_stop, /mission/command).
  Pour piloter depuis un autre appareil (tablette…), activer explicitement ihm_lan:=true,
  de préférence sur un réseau dédié au robot (point d'accès isolé ou VPN).

Usage :
  ros2 launch faucon_ihm ihm_launch.py                 # local uniquement
  ros2 launch faucon_ihm ihm_launch.py ihm_lan:=true   # accessible depuis le réseau
  ros2 launch faucon_ihm ihm_launch.py rosbridge_port:=9090 video_port:=8080 ihm_port:=3000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    video_port     = LaunchConfiguration("video_port")
    ihm_port       = LaunchConfiguration("ihm_port")

    lan = LaunchConfiguration("ihm_lan").perform(context).strip().lower() in ("true", "1", "yes")
    bind = "0.0.0.0" if lan else "127.0.0.1"
    # false : l'IHM est servie ailleurs (ex. conteneur nginx sous Docker)
    serve_ihm = LaunchConfiguration("ihm_server").perform(context).strip().lower() in ("true", "1", "yes")

    # ihm_host conservé pour compatibilité : "auto" = suit ihm_lan
    ihm_host = LaunchConfiguration("ihm_host").perform(context).strip()
    if ihm_host in ("", "auto"):
        ihm_host = bind

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": rosbridge_port,
            # "" = toutes les interfaces ; 127.0.0.1 = machine locale uniquement
            "address": "" if lan else "127.0.0.1",
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
            "address": bind,
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

    shown = "0.0.0.0 (RÉSEAU)" if lan else "127.0.0.1 (local)"
    log_start = LogInfo(msg=[
        "\n",
        "╔══════════════════════════════════════════════╗\n",
        "║        ROS2 IHM — GROUND CONTROL             ║\n",
        "╠══════════════════════════════════════════════╣\n",
        f"║  écoute      {shown}\n",
        "║  rosbridge   port ", rosbridge_port, "\n",
        "║  camera      port ", video_port, "\n",
        *(["║  IHM         port ", ihm_port, "\n"] if serve_ihm
          else ["║  IHM         servie ailleurs (ihm_server:=false)\n"]),
        "╚══════════════════════════════════════════════╝\n",
    ])

    actions = [log_start]
    if lan:
        actions.append(LogInfo(msg=(
            "[IHM] ⚠  ihm_lan:=true — rosbridge, la caméra et l'IHM sont accessibles depuis "
            "le réseau SANS authentification : tout appareil du réseau peut piloter le robot. "
            "À n'utiliser que sur un réseau dédié au robot."
        )))
    else:
        actions.append(LogInfo(msg=(
            "[IHM] Accès local uniquement. Pour piloter depuis un autre appareil : ihm_lan:=true"
        )))
    actions += [rosbridge_node, video_server_node]
    if serve_ihm:
        actions.append(ihm_server)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rosbridge_port", default_value="9090",
            description="Port WebSocket de rosbridge"
        ),
        DeclareLaunchArgument(
            "video_port", default_value="8080",
            description="Port HTTP de web_video_server"
        ),
        DeclareLaunchArgument(
            "ihm_port", default_value="3000",
            description="Port HTTP du serveur IHM"
        ),
        DeclareLaunchArgument(
            "ihm_lan", default_value="false",
            description="true : rosbridge, caméra et IHM accessibles depuis le réseau "
                        "(SANS authentification) ; false : machine locale uniquement"
        ),
        DeclareLaunchArgument(
            "ihm_host", default_value="auto",
            description="Interface d'écoute du serveur HTTP IHM ; 'auto' = suit ihm_lan"
        ),
        DeclareLaunchArgument(
            "ihm_server", default_value="true",
            description="false : ne pas lancer le serveur HTTP de l'IHM "
                        "(servie par un autre conteneur, ex. Docker)"
        ),
        OpaqueFunction(function=launch_setup),
    ])
