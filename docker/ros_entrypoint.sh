#!/usr/bin/env bash
# Point d'entree de l'image faucon-ros : environnement ROS + workspace Faucon.
set -e
source /opt/ros/jazzy/setup.bash
source /faucon_ws/install/setup.bash
exec "$@"
