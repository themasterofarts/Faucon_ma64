#!/usr/bin/env bash
# faucon/launch_base.sh

set -e
# set -u
set -o pipefail


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"   
cd "${WS_DIR}"

echo "Workspace : ${WS_DIR}"

use_mini=false
ihm_lan=false
headless=true
use_lidar_calibrator=false

# Parsing des arguments
for arg in "$@"; do
    case $arg in
        use_mini=*)
            use_mini="${arg#*=}"
            ;;
        ihm_lan=*)
            ihm_lan="${arg#*=}"
            ;;
        headless=*)
            headless="${arg#*=}"
            ;;
        use_lidar_calibrator=*)
            use_lidar_calibrator="${arg#*=}"
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

if [[ "$ihm_lan" != "true" && "$ihm_lan" != "false" ]]; then
    echo "Erreur : ihm_lan doit être true ou false (valeur reçue : $ihm_lan)"
    exit 1
fi
for v in headless use_lidar_calibrator; do
    if [[ "${!v}" != "true" && "${!v}" != "false" ]]; then
        echo "Erreur : $v doit être true ou false (valeur reçue : ${!v})"
        exit 1
    fi
done

if [[ "$ihm_lan" == "true" ]]; then
    echo "⚠  ihm_lan=true : IHM et rosbridge accessibles depuis le réseau SANS authentification."
fi

echo "Compilation (colcon build)…"
colcon build --symlink-install --event-handlers console_direct+ --parallel-workers "$(nproc)"
echo "Build terminé."

source "${WS_DIR}/install/setup.bash"


# Lancement
echo "Lancement : ros2 launch faucon_base_desc view.launch.py use_sim_time:=true"
exec ros2 launch faucon_base_desc view.launch.py use_sim_time:=true use_mini:="${use_mini}" ihm_lan:="${ihm_lan}" headless:="${headless}" use_lidar_calibrator:="${use_lidar_calibrator}"

