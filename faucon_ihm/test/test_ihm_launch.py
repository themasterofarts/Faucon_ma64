"""Tests de launch/ihm_launch.py : argument ihm_server.

ihm_server:=false (lancement Docker) : rosbridge et web_video_server tournent,
mais pas le serveur HTTP de l'IHM, servie par un autre conteneur (nginx).
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

_PATH = Path(__file__).resolve().parents[1] / "launch" / "ihm_launch.py"
_spec = importlib.util.spec_from_file_location("ihm_launch", _PATH)
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)


def actions_for(ihm_server):
    context = LaunchContext()
    context.launch_configurations.update({
        "rosbridge_port": "9090",
        "video_port": "8080",
        "ihm_port": "3000",
        "ihm_lan": "false",
        "ihm_host": "auto",
        "ihm_server": ihm_server,
    })
    return mod.launch_setup(context)


def names(actions):
    """Noms des Node / ExecuteProcess, lus avant execution (pas de contexte de lancement)."""
    out = set()
    for a in actions:
        if isinstance(a, Node):
            out.add(a._Node__node_name)  # node_name n'est resolu qu'a l'execution
        elif isinstance(a, ExecuteProcess) and a.name:
            out.add("".join(s.text for s in a.name))
    return out


def test_ihm_server_true_starts_http_server():
    assert "ihm_http_server" in names(actions_for("true"))


def test_ihm_server_false_keeps_rosbridge_and_video_only():
    acts = actions_for("false")
    assert "ihm_http_server" not in names(acts)
    assert {"rosbridge_websocket", "web_video_server"} <= names(acts)


def test_ihm_server_defaults_to_true():
    decl = [a for a in mod.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)]
    defaults = {a.name: a.default_value[0].text for a in decl}
    assert defaults["ihm_server"] == "true"
