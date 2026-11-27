# Faucon sous Docker (optionnel)

Ce répertoire permet de lancer Faucon **sans installer ROS 2 Jazzy, Gazebo ni Nav2** sur
sa machine. L'installation native (voir le [README principal](../README.md)) reste la voie
principale, notamment pour développer : Docker est une alternative pour essayer le projet
ou faire une démonstration.

Deux images :

| Image | Contenu | Taille |
|---|---|---|
| `faucon-ros` | ROS 2 Jazzy, Gazebo, Nav2, robot_localization, RTAB-Map, rosbridge, web_video_server et le workspace Faucon compilé | ~5,4 Go |
| `faucon-ihm` | l'IHM web React compilée, servie par nginx | ~70 Mo |

## Prérequis

- **Linux** : les conteneurs utilisent le réseau de l'hôte (`network_mode: host`), qui
  n'existe pas sous macOS ni Windows.
- **Docker ≥ 24 avec Compose v2** (`docker compose version`).
- *Optionnel* : un GPU NVIDIA avec [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
  Il est utilisé automatiquement s'il est détecté (rendu de Gazebo et des capteurs simulés).
  Sans GPU, le rendu se fait sur le CPU : la simulation fonctionne, en plus lent.

## Utilisation

Depuis la racine du dépôt :

```bash
./docker/faucon_docker.sh build                 # construit les deux images (longue la première fois : plusieurs Go à télécharger)
./docker/faucon_docker.sh base                  # simulation (équivalent de faucon/launch_base.sh)
./docker/faucon_docker.sh full                  # simulation + navigation (équivalent de launch_full.sh)
./docker/faucon_docker.sh full headless=false   # avec les fenêtres Gazebo et RViz
./docker/faucon_docker.sh logs                  # suit les journaux (depuis un autre terminal)
./docker/faucon_docker.sh down                  # arrête et supprime les conteneurs
```

`base` et `full` construisent les images au premier lancement si elles n'existent pas.
**Ctrl+C** arrête proprement la simulation et supprime les conteneurs (jusqu'à ~15 s).

Puis ouvrir l'IHM : **http://127.0.0.1:3000**.

### Options

Les mêmes que les scripts natifs, avec les mêmes valeurs par défaut :

| Option | Défaut | Rôle |
|---|---|---|
| `use_mini=true` | `false` | robot mini (DiffDrive Gazebo) |
| `headless=false` | `true` | affiche Gazebo (et RViz en mode `full`) |
| `use_lidar_calibrator=true` | `false` | lance `lidar_calibrator` (requis par RTAB-Map) |
| `ihm_lan=true` | `false` | IHM, rosbridge et caméra accessibles depuis le réseau |

Variables d'environnement :

| Variable | Défaut | Rôle |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | domaine ROS 2 des conteneurs |
| `FAUCON_DOCKER_GPU` | `auto` | `1` force le GPU NVIDIA, `0` le désactive |
| `FAUCON_DOCKER_DRY_RUN` | `0` | `1` affiche les commandes docker sans les exécuter |

### Ports

| Port | Service | Conteneur |
|---|---|---|
| 3000 | IHM web | `faucon-ihm` (nginx) |
| 9090 | rosbridge (WebSocket) | `faucon-ros` |
| 8080 | flux caméra (web_video_server) | `faucon-ros` |

Par défaut, tout écoute **uniquement sur 127.0.0.1**. Avec `ihm_lan=true`, ces services
sont ouverts au réseau **sans authentification** : tout appareil du réseau peut piloter le
robot. À n'utiliser que sur un réseau dédié au robot.

## Interagir avec ROS dans le conteneur

```bash
docker compose -f docker/compose.yaml exec ros bash    # shell avec l'environnement ROS chargé
ros2 topic list
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

Comme les conteneurs partagent le réseau de l'hôte, un ROS 2 Jazzy installé sur la machine
voit aussi les topics (même `ROS_DOMAIN_ID`).

## Après une modification du code

Le code est **copié dans l'image** au moment de la construction : après une modification,
reconstruire avant de relancer.

```bash
./docker/faucon_docker.sh build
```

Seule la compilation est refaite (~1 min) : les dépendances sont dans une couche mise en
cache, invalidée uniquement si un `package.xml` change.

## Fichiers

| Fichier | Rôle |
|---|---|
| `faucon_docker.sh` | point d'entrée : valide les options et pilote `docker compose` |
| `compose.yaml` | services `ros` et `ihm` |
| `compose.gpu.yaml` | ajouté si un GPU NVIDIA est disponible (y compris portables hybrides PRIME) |
| `Dockerfile.ros`, `Dockerfile.ihm` | les deux images |
| `*.dockerignore` | listes blanches : les rosbags et artefacts locaux ne sont jamais envoyés à Docker |
| `ros_entrypoint.sh` | charge l'environnement ROS au démarrage du conteneur |
| `nginx/default.conf.template` | configuration du serveur de l'IHM |
| `tests/` | tests du script (`./tests/run_tests.sh unit`), sans Docker |

## Limites

- Pas de montage des sources : pour développer, utiliser l'installation native.
- Linux uniquement (réseau de l'hôte).
- `headless=false` utilise le serveur X de l'hôte : le script autorise l'utilisateur `root`
  du conteneur à s'y connecter (`xhost +si:localuser:root`).
