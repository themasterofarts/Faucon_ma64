"""Tests de launch/view.launch.py : chargement des controleurs ros2_control.

Trois spawners separes echouaient parfois au demarrage sous charge (pas de reponse de
controller_manager a load_controller en 10 s) : velocity_controller restait non
configure, les roues ne recevaient rien et Nav2 abandonnait ("Failed to make progress")
sans autre signe. Un seul spawner, des delais longs, et un echec qui arrete le lancement.
"""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

_PATH = Path(__file__).resolve().parents[1] / "launch" / "view.launch.py"
CONTROLLERS = ["joint_broad", "steer_controller", "velocity_controller"]


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("view_launch", _PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def actions(mod):
    context = LaunchContext()
    for a in mod.generate_launch_description().entities:
        if isinstance(a, DeclareLaunchArgument):
            context.launch_configurations[a.name] = "".join(s.text for s in a.default_value)
    try:
        return mod.launch_setup(context)
    except Exception as exc:  # paquets non installes (workspace non source)
        pytest.skip(f"view.launch.py non chargeable : {exc}")


def _args(node):
    return [_text(a) for a in node._Node__arguments]


def _text(value):
    if isinstance(value, str):
        return value
    return "".join(getattr(s, "text", "") for s in value)


def spawners(actions):
    return [a for a in actions if isinstance(a, Node)
            and _text(a._Node__node_executable) == "spawner"]


def test_single_spawner_loads_all_controllers(actions):
    sp = spawners(actions)
    assert len(sp) == 1
    args = _args(sp[0])
    assert args[:3] == CONTROLLERS


def test_spawner_waits_long_enough_under_load(actions):
    args = _args(spawners(actions)[0])
    for opt, minimum in (("--controller-manager-timeout", 60), ("--service-call-timeout", 30)):
        assert opt in args
        assert float(args[args.index(opt) + 1]) >= minimum


def test_spawner_failure_stops_the_launch(actions):
    sp = spawners(actions)[0]
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)
                and isinstance(a.event_handler, OnProcessExit)
                and a.event_handler._OnActionEventBase__action_matcher is sp]
    assert len(handlers) == 1


@pytest.mark.parametrize("code,stops", [(1, True), (0, False)])
def test_shutdown_only_when_spawner_fails(actions, code, stops):
    from types import SimpleNamespace
    from launch.actions import Shutdown
    sp = spawners(actions)[0]
    handler = next(a.event_handler for a in actions if isinstance(a, RegisterEventHandler)
                   and a.event_handler._OnActionEventBase__action_matcher is sp)
    result = handler._OnActionEventBase__on_event(SimpleNamespace(returncode=code), LaunchContext())
    assert any(isinstance(r, Shutdown) for r in result or []) is stops
