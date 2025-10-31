#!/usr/bin/env bash
# faucon/launch_base.sh

set -e
set -u
set -o pipefail


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"   
cd "${WS_DIR}"

echo "Workspace : ${WS_DIR}"

echo "Compilation (colcon build)…"
colcon build --symlink-install --event-handlers console_direct+ --parallel-workers "$(nproc)"
echo "Build terminé."


if [ -f "${WS_DIR}/install/setup.bash" ]; then
  set +u
  : "${COLCON_TRACE:=0}"
  source "${WS_DIR}/install/setup.bash"
  set -u
  echo "🔗 Environnement sourcé."
else
  echo " ${WS_DIR}/install/setup.bash introuvable." >&2
  exit 1
fi

# Lancement
echo "Lancement : ros2 launch faucon_base_desc view.launch.py use_sim_time:=true"
exec ros2 launch faucon_base_desc view.launch.py use_sim_time:=true
