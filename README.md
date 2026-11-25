# 🦅 Projet Faucon

<p align="center">
  
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white" alt="ROS2 Jazzy"/>
  <img src="https://img.shields.io/badge/Simulator-Gazebo-orange?logo=ros&logoColor=white" alt="Gazebo"/>
  <img src="https://img.shields.io/badge/Control-PX4-blueviolet?logo=drone&logoColor=white" alt="PX4"/>
  <img src="https://img.shields.io/badge/BehaviorTree-CPP-green" alt="BehaviorTree.CPP"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow" alt="MIT License"/>
  <img src="https://img.shields.io/badge/Contributions-Welcome-success" alt="Contributions Welcome"/>
  <a href="https://discord.gg/CEVwVY9RJy">
    <img src="https://img.shields.io/discord/000000000000000000?color=7289DA&label=Discord&logo=discord&logoColor=white" alt="Discord MH64 Robotics"/>
  </a>
</p>

[![CI – develop](https://github.com/themasterofarts/Faucon_ma64/actions/workflows/ci.yaml/badge.svg?branch=develop)](https://github.com/themasterofarts/Faucon_ma64/actions/workflows/ci.yaml?query=branch%3Adevelop)
[![CI – main](https://github.com/themasterofarts/Faucon_ma64/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/themasterofarts/Faucon_ma64/actions/workflows/ci.yaml?query=branch%3Amain)

---

###  À propos

**Faucon** est un projet open-source initié par la communauté **MA64 Robotics**, composée de passionnés de **robotique** et d’**intelligence artificielle**.  
L’objectif est de développer une **stack complète pour l’agriculture intelligente**, reposant sur la **collaboration entre un robot mobile et un drone** évoluant dans un champ.

Le projet sert à la fois de **plateforme de recherche**, de **base d’expérimentation** et de **projet communautaire** pour les membres souhaitant apprendre, contribuer et innover autour des technologies ROS2.

---

### Technologies principales

| Composant | Description |
|------------|-------------|
| **ROS2 Jazzy** | Middleware principal pour la robotique |
| **Gazebo Sim** | Simulation 3D du robot et du drone |
| **Nav2** | Navigation autonome du robot mobile |
| **BehaviorTree.CPP** | Décision et orchestration de comportements |
| **PX4** | Contrôleur de vol pour le drone |
| **Python / C++** | Langages principaux utilisés |

---
### 🆕 Nouveautés récentes

- **Cartographie 3D fusionnée LiDAR + RGBD (`faucon_perception`)** : deux modes RTAB-Map disponibles — LiDAR seul (ICP) et fusion LiDAR + caméra profondeur (visual+ICP). Le nuage LiDAR passe par `lidar_calibrator` (RANSAC sol) avant d’être injecté dans RTAB-Map.
- **Caméra RGBD unifiée** : passage au type `rgbd_camera` Gazebo, pipeline image corrigé (`/camera/image`, `/camera/depth_image`). Un seul bridge `/clock` actif (suppression du doublon qui provoquait des sauts TF).
- **Stabilité Nav2 améliorée** : `transform_tolerance` porté à 0.5 s dans le controller et les deux costmaps. EKF avec `smooth_lagged_data` pour absorber les données hors-ordre.
- **Script de lancement complet (`launch_full.sh`)** : lance automatiquement la base et la navigation en une seule commande, avec build, synchronisation et arrêt propre.
- **IHM web ROS2 intégrée (`faucon_ihm`)** : dashboard React (GPS, IMU, caméra, télémétrie, contrôle manuel) désormais inclus dans le workspace et lancé automatiquement avec la simulation principale.
- **Support Mapviz/OpenStreetMap (`faucon_localisation`)** : ajout d’un lancement dédié pour visualiser la position GNSS sur carte.
- **Mode mini robot amélioré** : argument `use_mini` pris en charge dans le script de lancement global et dans le launch principal (adaptation du modèle/contrôle).

---


### 🧩 Architecture du projet
```bash 
Faucon_ma64/
├── Faucon_Orchestration/ # Coordination robot + drone via Behavior Trees
├── faucon/ # Scripts principaux, lancement global
├── faucon_base_desc/ # Description URDF/Xacro du robot
├── faucon_control/ # Contrôleurs 4WS, plugins ROS2 Control
├── faucon_drone/ # Intégration et simulation du drone PX4
├── faucon_localisation/ # Fusion GPS, IMU, odométrie
├── faucon_ihm/ # Interface web (dashboard + commande manuelle)
├── faucon_navigation/ # Stack Nav2 et configuration
├── faucon_perception/ # Traitement des capteurs et vision
├── virtual_maize_field/ # Environnement de simulation agricole
└── README.md

```
---

### Lancement rapide

#### **1. Cloner le projet**
```bash
git clone git@github.com:themasterofarts/Faucon_ma64.git
cd Faucon_ma64

rosdep update
rosdep install --from-paths . --ignore-src -y
```

---

#### **2a. Lancement base uniquement** (simulation + RViz2)
```bash
sudo chmod +x ./faucon/launch_base.sh
./faucon/launch_base.sh
```

#### Variante mini robot
```bash
./faucon/launch_base.sh use_mini=true
```

---

#### **2b. Lancement complet** (base + navigation autonome)

`launch_full.sh` enchaîne automatiquement :
1. `colcon build --symlink-install`
2. `ros2 launch faucon_base_desc view.launch.py` (arrière-plan)
3. Attente 5 s, puis `ros2 launch faucon_navigation faucon_nav.launch.py` (arrière-plan)
4. Arrêt propre des deux processus sur Ctrl+C

```bash
sudo chmod +x ./faucon/launch_full.sh
./faucon/launch_full.sh
```

#### Variante mini robot
```bash
./faucon/launch_full.sh use_mini=true
```

---

#### Options de lancement (`launch_base.sh` et `launch_full.sh`)

| Option | Défaut | Effet |
|---|---|---|
| `use_mini=true` | `false` | Robot mini (DiffDrive) |
| `headless=false` | `true` | Affiche l'interface Gazebo. Par défaut Gazebo tourne **sans interface** (capteurs toujours simulés), ce qui économise ~2 cœurs et ~0,9 Go ; RViz ou l'IHM servent à visualiser. |
| `use_lidar_calibrator=true` | `false` | Lance `lidar_calibrator` (`/cloud_calib_out`), **requis par la cartographie RTAB-Map et la détection de rangs** |
| `ihm_lan=true` | `false` | IHM accessible depuis le réseau (voir plus bas) |

```bash
./faucon/launch_full.sh headless=false                 # avec l'interface Gazebo
./faucon/launch_full.sh use_lidar_calibrator=true      # avant une cartographie
```

---

#### **3. Cartographie RTAB-Map** (optionnel, après `launch_full.sh use_lidar_calibrator=true`)

> La cartographie utilise `/cloud_calib_out`, publié par `lidar_calibrator`, qui n'est plus
> lancé par défaut : lancer la base avec `use_lidar_calibrator=true`.

```bash
# LiDAR seul
ros2 launch faucon_perception mapping_3d.launch.py

# LiDAR + caméra RGBD (fusion visuelle + ICP)
ros2 launch faucon_perception mapping_rgbd.launch.py

# reprendre une session existante
ros2 launch faucon_perception mapping_rgbd.launch.py fresh_start:=false open_viz:=true
```

---

#### Lancer Mapviz (visualisation GPS)
```bash
ros2 launch faucon_localisation mapviz.launch.py
```

#### Lancer uniquement l'IHM (hors lancement global)
```bash
ros2 launch faucon_ihm ihm_launch.py
```

#### Accès à l'IHM depuis un autre appareil (tablette…)
Par défaut, l'IHM, rosbridge et la caméra n'écoutent **qu'en local** (`http://localhost:3000`).
rosbridge n'a **pas d'authentification** : l'ouvrir au réseau permet à tout appareil du réseau
de piloter le robot. Pour l'activer explicitement :
```bash
./faucon/launch_full.sh ihm_lan=true          # ou launch_base.sh ihm_lan=true
ros2 launch faucon_ihm ihm_launch.py ihm_lan:=true
```
puis ouvrir `http://<IP-robot>:3000`. À n'utiliser que sur un réseau dédié au robot
(point d'accès isolé ou VPN).

#### Pilotage en manuel
```bash
# remap vers l'entree teleop de twist_mux (prioritaire sur Nav2)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

> Les commandes de vitesse passent par `twist_mux` : Nav2 publie sur `/cmd_vel_nav2` (priorité 10),
> la téléopération (IHM, clavier) sur `/cmd_vel_teleop` (priorité 100). `twist_mux` publie le résultat sur `/cmd_vel`.
> Ne publiez pas directement sur `/cmd_vel` : cela contournerait l'arbitrage.
---

### Tests

Les tests se lancent en local comme dans la CI, depuis la racine du dépôt :

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # après colcon build
./tests/run_tests.sh              # unitaires + intégration + rapport de couverture
./tests/run_tests.sh unit         # tests unitaires seuls (rapides)
./tests/run_tests.sh integration  # tests d'intégration seuls
```

- **Tests unitaires** (`*/test/`) : logique pure (cinématique 4WS, générateur de trajectoires, validation de mission, URDF, utilitaires).
- **Tests d'intégration** (`tests/integration/`) : vrais nœuds ROS 2, **sans Gazebo**. Ils couvrent la chaîne de sécurité (`twist_mux` + `control_4ws` / `diffdrive_safety` : priorités, délai de sécurité, arrêt d'urgence) et le cycle de vie d'une mission (`mission_manager` + `datum_manager`, avec Nav2 et navsat_transform factices). Ils utilisent un domaine ROS dédié et une découverte limitée à la machine : aucune interférence avec une simulation en cours.
- **Couverture** : rapport HTML dans `coverage_html/index.html`. En CI, le tableau de couverture apparaît dans le résumé du job « Coverage report », et le rapport complet est disponible en artefact.

### Contribuer au projet

Nous accueillons toutes les contributions de la communauté !

#### Processus de contribution

Fork du dépôt (ou clone si membre de l’équipe principale).

Crée une nouvelle branche à partir de develop :

```bash
git checkout develop
git pull
git checkout -b feat/ma_fonctionnalite

git commit -m "feat: ajout du module de perception lidar"

git push origin feat/ma_fonctionnalite


```

#### Crée une Pull Request vers develop.
Chaque fusion sur develop ou main passe obligatoirement par une revue via PR.

### Rythme du projet

Le développement est communautaire et évolutif.
Des réunions de coordination sont organisées régulièrement pour planifier les évolutions et accueillir de nouveaux contributeurs.

### 💬 Rejoins le canal #projet-faucon sur le Discord MA64 Robotics


---

### Licences

Le projet est distribué sous licence **MIT**, avec les exceptions suivantes :

| Paquet | Licence | Remarque |
|---|---|---|
| `faucon_base_desc`, `faucon_control`, `faucon_drone`, `faucon_ihm`, `faucon_navigation`, `faucon_perception`, `Faucon_Orchestration` | MIT | |
| `faucon_localisation` | Apache-2.0 | Contient du code contribué sous Apache-2.0 ; un passage en MIT nécessite l'accord de ses contributeurs. |
| `virtual_maize_field` | GPLv3 | Projet externe embarqué (Field Robot Event). Sa licence s'applique à ce paquet et à toute redistribution qui l'inclut. |

La licence de chaque paquet fait foi dans son `package.xml` (et son fichier `LICENSE` lorsqu'il existe).

### Remerciements

Un grand merci à tous les membres de la communauté MA64 Robotics pour leur implication, leurs idées et leur passion pour la robotique.
Ensemble, faisons de Faucon un projet de référence en robotique agricole open source ! 🚜🤖
