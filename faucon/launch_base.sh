#!/usr/bin/env bash
# faucon/launch_base.sh

# set -e
# set -u
# set -o pipefail


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"   
cd "${WS_DIR}"

echo "Workspace : ${WS_DIR}"

echo "Compilation (colcon build)…"
colcon build --symlink-install --event-handlers console_direct+ --parallel-workers "$(nproc)"
echo "Build terminé."

source "${WS_DIR}/install/setup.bash"


# Lancement
echo "Lancement : ros2 launch faucon_base_desc view.launch.py use_sim_time:=true"
exec ros2 launch faucon_base_desc view.launch.py use_sim_time:=true
