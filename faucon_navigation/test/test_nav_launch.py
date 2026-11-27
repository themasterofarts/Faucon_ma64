"""Tests de launch/faucon_nav.launch.py : argument use_rviz.

Sans affichage (Docker headless), rviz2 s'arrete au demarrage ; le launch rviz de
nav2_bringup arrete alors TOUT le lancement (Shutdown a la sortie de rviz) :
use_rviz:=false doit donc pouvoir ne pas l'inclure.
"""
import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition

_PATH = Path(__file__).resolve().parents[1] / "launch" / "faucon_nav.launch.py"


@pytest.fixture(scope="module")
def entities():
    spec = importlib.util.spec_from_file_location("faucon_nav_launch", _PATH)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
        return mod.generate_launch_description().entities
    except Exception as exc:  # paquets non installes (workspace non source)
        pytest.skip(f"faucon_nav.launch.py non chargeable : {exc}")


def test_use_rviz_defaults_to_true(entities):
    decl = {a.name: a for a in entities if isinstance(a, DeclareLaunchArgument)}
    assert decl["use_rviz"].default_value[0].text == "true"


def test_rviz_include_is_conditioned_on_use_rviz(entities):
    def location(inc):  # texte du chemin, lu avant execution
        subs = inc.launch_description_source._LaunchDescriptionSource__location
        return "".join(getattr(s, "text", "") for s in subs)

    rviz = [a for a in entities if isinstance(a, IncludeLaunchDescription)
            and location(a).endswith("rviz_launch.py")]
    assert len(rviz) == 1
    cond = rviz[0].condition
    assert isinstance(cond, IfCondition)
    names = [getattr(s, "variable_name", None) for s in cond._IfCondition__predicate_expression]
    assert any(n and "".join(t.text for t in n) == "use_rviz" for n in names)
