#!/usr/bin/env bash
# faucon/launch_full.sh — lance la base + navigation

set -e
set -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"
cd "${WS_DIR}"

echo "Workspace : ${WS_DIR}"

use_mini=false

for arg in "$@"; do
    case $arg in
        use_mini=*)
            use_mini="${arg#*=}"
            ;;
        *)
            echo "Argument inconnu : $arg"
            exit 1
            ;;
    esac
done

if [[ "$use_mini" != "true" && "$use_mini" != "false" ]]; then
    echo "Erreur : use_mini doit être true ou false (valeur reçue : $use_mini)"
    exit 1
fi

echo "Compilation (colcon build)…"
colcon build --symlink-install --event-handlers console_direct+ --parallel-workers "$(nproc)"
echo "Build terminé."

source "${WS_DIR}/install/setup.bash"

# Lancement de la base en arrière-plan
echo "Lancement base : ros2 launch faucon_base_desc view.launch.py use_sim_time:=true use_mini:=${use_mini}"
ros2 launch faucon_base_desc view.launch.py use_sim_time:=true use_mini:="${use_mini}" &
BASE_PID=$!

echo "Attente du démarrage de la base (5s)…"
sleep 5

# Lancement de la navigation
echo "Lancement navigation : ros2 launch faucon_navigation faucon_nav.launch.py use_sim_time:=true"
ros2 launch faucon_navigation faucon_nav.launch.py use_sim_time:=true &
NAV_PID=$!

echo "Faucon complet lancé (base PID=${BASE_PID}, nav PID=${NAV_PID})"
echo "Ctrl+C pour arrêter les deux processus."

trap "echo 'Arrêt en cours…'; kill ${BASE_PID} ${NAV_PID} 2>/dev/null; wait" SIGINT SIGTERM

wait ${BASE_PID} ${NAV_PID}
