#!/usr/bin/env bash
# faucon/launch_base.sh

set -e
# set -u
set -o pipefail


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"   
cd "${WS_DIR}"

echo "Workspace : ${WS_DIR}"

# Kill any leftover Gazebo instance to ensure clean spawn
pkill -f gz_sim 2>/dev/null || true
pkill -f ruby.*gz 2>/dev/null || true
sleep 1

# Set CPU governor to performance for simulation (non-blocking, requires sudo)
if command -v cpupower &>/dev/null; then
    sudo -n cpupower frequency-set -g performance 2>/dev/null && echo "CPU governor → performance" || echo "CPU governor : sudo requis (ignoré)"
else
    echo performance | sudo -n tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null 2>&1 && echo "CPU governor → performance" || echo "CPU governor : sudo requis (ignoré)"
fi

use_mini=false

# Parsing des arguments
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


# Lancement
echo "Lancement : ros2 launch faucon_base_desc view.launch.py use_sim_time:=true"
exec ros2 launch faucon_base_desc view.launch.py use_sim_time:=true use_mini:="${use_mini}"

