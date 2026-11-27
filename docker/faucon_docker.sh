#!/usr/bin/env bash
# docker/faucon_docker.sh — lance Faucon via Docker (optionnel : l'installation
# native et faucon/launch_*.sh restent la voie principale).
#
#   ./docker/faucon_docker.sh build              construit les images faucon-ros et faucon-ihm
#   ./docker/faucon_docker.sh base [args…]       equivalent de faucon/launch_base.sh
#   ./docker/faucon_docker.sh full [args…]       equivalent de faucon/launch_full.sh
#   ./docker/faucon_docker.sh down               arrete et supprime les conteneurs
#   ./docker/faucon_docker.sh logs               suit les journaux
#
# Arguments (base / full) : use_mini= ihm_lan= headless= use_lidar_calibrator=
# (true|false), comme les scripts natifs.
#
# Variables : ROS_DOMAIN_ID (defaut 0) ; FAUCON_DOCKER_GPU=auto|0|1 (defaut auto :
# GPU NVIDIA si le runtime nvidia est installe) ; FAUCON_DOCKER_DRY_RUN=1 affiche
# les commandes docker sans les executer.

set -euo pipefail

DOCKER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DRY_RUN="${FAUCON_DOCKER_DRY_RUN:-0}"

usage() {
    sed -n '4,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    echo "usage : $0 {build|base|full|down|logs} [use_mini=…] [ihm_lan=…] [headless=…] [use_lidar_calibrator=…]" >&2
    exit 2
}

# Affiche (simulation) ou execute une commande.
run() {
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "+ $*"
    else
        "$@"
    fi
}

[[ $# -ge 1 ]] || usage
sub="$1"
shift
case "$sub" in
    build|base|full|down|logs) ;;
    *) usage ;;
esac

use_mini=false
ihm_lan=false
headless=true
use_lidar_calibrator=false

for arg in "$@"; do
    case $arg in
        use_mini=*) use_mini="${arg#*=}" ;;
        ihm_lan=*) ihm_lan="${arg#*=}" ;;
        headless=*) headless="${arg#*=}" ;;
        use_lidar_calibrator=*) use_lidar_calibrator="${arg#*=}" ;;
        *)
            echo "Argument inconnu : $arg"
            exit 1
            ;;
    esac
done

for v in use_mini ihm_lan headless use_lidar_calibrator; do
    if [[ "${!v}" != "true" && "${!v}" != "false" ]]; then
        echo "Erreur : $v doit être true ou false (valeur reçue : ${!v})"
        exit 1
    fi
done

compose=(docker compose -f "${DOCKER_DIR}/compose.yaml")

# GPU NVIDIA : rendu Gazebo et capteurs simules (camera, LiDAR), headless ou non.
gpu="${FAUCON_DOCKER_GPU:-auto}"
if [[ "$gpu" == "auto" ]]; then
    if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
        gpu=1
    else
        gpu=0
    fi
fi

case "$sub" in
    build) run "${compose[@]}" build; exit 0 ;;
    down)  run "${compose[@]}" down; exit 0 ;;
    logs)  run "${compose[@]}" logs -f; exit 0 ;;
esac

# --- base / full ---
if [[ "$ihm_lan" == "true" ]]; then
    echo "⚠  ihm_lan=true : IHM et rosbridge accessibles depuis le réseau SANS authentification."
    export IHM_BIND=0.0.0.0
else
    export IHM_BIND=127.0.0.1
fi

base_cmd="ros2 launch faucon_base_desc view.launch.py use_sim_time:=true use_mini:=${use_mini} ihm_lan:=${ihm_lan} headless:=${headless} use_lidar_calibrator:=${use_lidar_calibrator} ihm_server:=false"
if [[ "$sub" == "base" ]]; then
    FAUCON_ROS_CMD="exec ${base_cmd}"
else
    # meme sequence que faucon/launch_full.sh : la base, 5 s, puis la navigation
    # RViz seulement avec affichage : sans display, rviz2 s'arrete et arrete toute la navigation
    if [[ "$headless" == "false" ]]; then use_rviz=true; else use_rviz=false; fi
    FAUCON_ROS_CMD="${base_cmd} & BASE_PID=\$!; sleep 5; ros2 launch faucon_navigation faucon_nav.launch.py use_sim_time:=true use_rviz:=${use_rviz} & NAV_PID=\$!; trap 'kill -INT \$BASE_PID \$NAV_PID 2>/dev/null; wait' INT TERM; wait"
fi
export FAUCON_ROS_CMD

if [[ "$gpu" == "1" ]]; then
    compose+=(-f "${DOCKER_DIR}/compose.gpu.yaml")
fi

if [[ "$headless" == "true" ]]; then
    # aucune application graphique dans le conteneur
    export DISPLAY=
else
    # autorise le conteneur (root) a afficher l'interface Gazebo sur le serveur X local
    if command -v xhost >/dev/null 2>&1; then
        run xhost +si:localuser:root
    else
        echo "xhost introuvable : l'affichage de Gazebo depuis le conteneur risque d'être refusé."
    fi
fi

# Construction au premier lancement (images absentes)
present="${FAUCON_DOCKER_IMAGES_PRESENT:-}"
if [[ -z "$present" ]]; then
    if docker image inspect faucon-ros faucon-ihm >/dev/null 2>&1; then present=1; else present=0; fi
fi
up_args=(up --abort-on-container-exit)
[[ "$present" == "1" ]] || up_args+=(--build)

echo "Lancement ${sub} : IHM http://127.0.0.1:3000 (écoute ${IHM_BIND}), rosbridge :9090, caméra :8080"
if [[ "$DRY_RUN" == "1" ]]; then
    echo "  IHM_BIND=${IHM_BIND}"
    echo "  DISPLAY=${DISPLAY:-}"
    echo "  FAUCON_ROS_CMD=${FAUCON_ROS_CMD}"
fi

cleanup() {
    trap - EXIT INT TERM
    echo "Arrêt des conteneurs…"
    run "${compose[@]}" down
}
trap cleanup EXIT

if [[ "$DRY_RUN" == "1" ]]; then
    run "${compose[@]}" "${up_args[@]}"
    exit 0
fi

# compose up tourne en arriere-plan : bash differe les trap tant qu'une commande au
# premier plan s'execute ; avec wait, Ctrl+C / kill declenchent l'arret immediatement.
"${compose[@]}" "${up_args[@]}" &
up_pid=$!
# signal transmis a compose up (SIGTERM : arret propre des conteneurs comme Ctrl+C ;
# SIGINT est ignore par les taches d'arriere-plan d'un bash non interactif), puis down
trap 'kill -TERM "$up_pid" 2>/dev/null; wait "$up_pid" 2>/dev/null; exit 130' INT TERM
status=0
wait "$up_pid" || status=$?
exit "$status"
