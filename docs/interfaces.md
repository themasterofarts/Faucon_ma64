# Faucon — Interfaces ROS2

Ce document décrit toutes les interfaces du stack Faucon : ce que le robot doit fournir, ce que les modules publient entre eux, et ce que l'opérateur (IHM / partenaire) peut consommer.

---

## Vue d'ensemble du flux de données

```
┌─────────────────────────────────────────────────────────────────────┐
│                        HARDWARE / SIMULATEUR                        │
│   /gnss/fix  /imu/data  /odom  /cloud  /scan  /camera/*            │
└─────────────────────────┬───────────────────────────────────────────┘
                          │  (topics capteurs — contrat robot)
          ┌───────────────▼────────────────┐
          │        faucon_localisation      │
          │  datum_manager + dual EKF +     │
          │  navsat_transform               │
          │  → odometry/local               │
          │  → odometry/global              │
          │  → /gnss/datum                  │
          └───────────────┬────────────────┘
                          │
          ┌───────────────▼────────────────┐     ┌──────────────────────────────┐
          │        faucon_perception        │     │       faucon_navigation       │
          │  lidar_calibrator               │     │  mission_manager             │
          │  → /cloud_calib_out             │     │  + Nav2 (FollowPath)         │
          │                                 │     │  → /mission/status           │
          │  rtabmap_slam                   │     │  → /mission/path             │
          │  → /rtabmap/cloud_map           │     │  → /cmd_vel (via Nav2)       │
          │  → /rtabmap/map                 │     └──────────────────────────────┘
          └─────────────────────────────────┘
                          │
          ┌───────────────▼────────────────┐
          │          faucon_control         │
          │  control_4ws                    │
          │  /cmd_vel → steer + velocity    │
          │  controllers (ros2_control)     │
          └─────────────────────────────────┘
                          │
          ┌───────────────▼────────────────┐
          │           faucon_ihm            │
          │  rosbridge :9090                │
          │  web_video_server :8080         │
          │  IHM React :3000                │
          └─────────────────────────────────┘
```

---

## 1. Contrat robot — ce que le hardware doit publier

Pour que Faucon fonctionne sur un nouveau robot, celui-ci doit publier ces topics.
C'est le seul point de couplage entre Faucon et le hardware.

| Topic | Type | Fréquence recommandée | Description |
|-------|------|-----------------------|-------------|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | ≥ 5 Hz | Fix GNSS (lat, lon, altitude). Status ≥ 0 requis. |
| `/imu/data` | `sensor_msgs/Imu` | ≥ 50 Hz | IMU avec orientation et angular_velocity. |
| `/odom` | `nav_msgs/Odometry` | ≥ 20 Hz | Odométrie roues ou encodeurs. Frame: `odom` → `base_link`. |
| `/cloud` | `sensor_msgs/PointCloud2` | ≥ 5 Hz | Nuage de points LiDAR brut. N'importe quel frame. |
| `/scan` | `sensor_msgs/LaserScan` | ≥ 10 Hz | Scan 2D laser (utilisé par le costmap Nav2). |
| `/camera/image` | `sensor_msgs/Image` | optionnel | Flux caméra RGB (pour l'IHM). |
| `/camera/depth_image` | `sensor_msgs/Image` | optionnel | Profondeur caméra (pour mapping RGBD optionnel). |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | optionnel | Calibration caméra (sync avec depth_image). |
| `joint_states` | `sensor_msgs/JointState` | ≥ 20 Hz | État des joints (requis par robot_state_publisher). |
| `tf` | `tf2_msgs/TFMessage` | continu | Transformées hardware (ex: lidar_frame → base_link). |

> **Frames TF attendues** : le robot doit publier la chaîne `base_link` → tous ses capteurs (ex: `lidar_frame`, `imu_link`, `camera_link`). Le stack Faucon gère `map` → `odom` → `base_link`.

---

## 2. Module faucon_localisation

### Nœud : datum_manager

**Rôle** : capte le premier fix GNSS valide et fixe l'origine du repère local (datum). Ne s'exécute qu'une seule fois au démarrage.

| Direction | Topic / Service | Type | Description |
|-----------|----------------|------|-------------|
| Souscrit | `/gnss/fix` | `sensor_msgs/NavSatFix` | Premier fix valide |
| Publie | `/gnss/datum` | `sensor_msgs/NavSatFix` | Datum fixé (TRANSIENT_LOCAL) |
| Appelle | `/datum` | `robot_localization/SetDatum` | Ancre navsat_transform |

### Nœud : ekf_filter_node_odom

**Rôle** : fusion odom + IMU → odométrie locale lisse et continue (court terme, haute fréquence).

| Direction | Topic | Type | Frame world |
|-----------|-------|------|-------------|
| Souscrit | `/odom` | `nav_msgs/Odometry` | — |
| Souscrit | `/imu/data` | `sensor_msgs/Imu` | — |
| Publie | `odometry/local` | `nav_msgs/Odometry` | `odom` |

### Nœud : ekf_filter_node_map

**Rôle** : fusion odom + IMU + GPS → position absolue ancrée sur le datum GNSS (long terme).

| Direction | Topic | Type | Frame world |
|-----------|-------|------|-------------|
| Souscrit | `/odom` | `nav_msgs/Odometry` | — |
| Souscrit | `/imu/data` | `sensor_msgs/Imu` | — |
| Souscrit | `odometry/gps` | `nav_msgs/Odometry` | — |
| Publie | `odometry/global` | `nav_msgs/Odometry` | `map` |
| Publie | `tf` : `map` → `odom` | — | — |

### Nœud : navsat_transform

**Rôle** : convertit les coordonnées GPS (lat/lon) en coordonnées cartésiennes ENU dans le repère `map`.

| Direction | Topic / Service | Type | Description |
|-----------|----------------|------|-------------|
| Souscrit | `/gnss/fix` | `sensor_msgs/NavSatFix` | Fix GPS brut |
| Souscrit | `/imu/data` | `sensor_msgs/Imu` | Cap robot |
| Souscrit | `odometry/global` | `nav_msgs/Odometry` | Pose fusionnée |
| Publie | `odometry/gps` | `nav_msgs/Odometry` | GPS converti en ENU |
| Publie | `gps/filtered` | `sensor_msgs/NavSatFix` | GPS filtré |
| Expose | `/fromLLArray` | `robot_localization/FromLLArray` | Convertit liste GPS → ENU |
| Expose | `/datum` | `robot_localization/SetDatum` | Fixe l'origine |

---

## 3. Module faucon_perception

### Nœud : lidar_calibrator (LidarCalibrator)

**Rôle** : filtre le sol du nuage LiDAR par RANSAC et reprojette dans le frame `base_link`.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Souscrit | `/cloud` | `sensor_msgs/PointCloud2` | LiDAR brut |
| Publie | `/cloud_calib_out` | `sensor_msgs/PointCloud2` | Nuage sans le sol |
| Publie | `/ground_cloud` | `sensor_msgs/PointCloud2` | Plan sol détecté |

**Paramètres ROS2** :

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `ransac_distance_threshold` | `0.07` m | Tolérance pour classifier un point comme sol |
| `ransac_max_iterations` | `100` | Nombre max d'itérations RANSAC |

### Nœud : rtabmap (SLAM 3D)

**Rôle** : cartographie 3D et localisation simultanée (LiDAR seul).

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Souscrit | `scan_cloud` → `/cloud_calib_out` | `sensor_msgs/PointCloud2` | Nuage filtré |
| Souscrit | `odom` → `odometry/local` | `nav_msgs/Odometry` | Odométrie locale |
| Publie | `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | Carte 3D globale |
| Publie | `/rtabmap/map` | `nav_msgs/OccupancyGrid` | Carte 2D pour Nav2 |
| Publie | `/rtabmap/mapData` | `rtabmap_msgs/MapData` | Données internes RTAB-Map |

**Paramètres launch notables** :

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `database_path` | `~/.ros/faucon_map2.db` | Fichier de la carte persistante |
| `fresh_start` | `true` | Efface la carte précédente au démarrage |
| `open_viz` | `false` | Ouvre le visualiseur RTAB-Map |

---

## 4. Module faucon_navigation

### Nœud : mission_manager

**Rôle** : gère le cycle de vie complet d'une mission autonome (chargement, démarrage, pause, reprise, arrêt).

#### Topics consommés (commandes opérateur / IHM)

| Topic | Type | Description |
|-------|------|-------------|
| `/mission/load_path` | `std_msgs/String` | Contenu YAML de la mission (liste de waypoints GPS) |
| `/mission/command` | `std_msgs/String` | Commande : `START` / `STOP` / `PAUSE` / `RESUME` |

#### Topics consommés (état du robot)

| Topic | Type | Description |
|-------|------|-------------|
| `/gnss/datum` | `sensor_msgs/NavSatFix` | Confirme que la localisation est initialisée |
| `odometry/global` | `nav_msgs/Odometry` | Pose courante du robot en frame `map` |

#### Topics publiés

| Topic | Type | Fréquence | Description |
|-------|------|-----------|-------------|
| `/mission/status` | `std_msgs/String` | 5 Hz | JSON : état, mission_id, progression, erreur |
| `/mission/path` | `nav_msgs/Path` | événement (TRANSIENT_LOCAL) | Chemin ENU complet de la mission |

**Format `/mission/status`** :
```json
{
  "state":      "IDLE | LOADING | READY | RUNNING | PAUSED | COMPLETED | ABORTED | ERROR",
  "mission_id": "A3F2B1C0",
  "progress":   0.42,
  "current_wp": 5,
  "total_wp":   12,
  "error":      ""
}
```

**Format `/mission/load_path`** (YAML dans un String) :
```yaml
waypoints:
  - latitude: 43.899967
    longitude: 3.199972
    yaw: 0.0
  - latitude: 43.900100
    longitude: 3.200100
    yaw: 1.57
```

#### Action utilisée

| Action | Type | Description |
|--------|------|-------------|
| `/follow_path` | `nav2_msgs/action/FollowPath` | Envoi du chemin ENU à Nav2 |

#### Service utilisé

| Service | Type | Description |
|---------|------|-------------|
| `/fromLLArray` | `robot_localization/FromLLArray` | Conversion batch GPS → ENU |

### Nav2 (navigation stack)

Nav2 est lancé via `nav2_bringup`. Les interfaces principales :

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Souscrit | `odometry/local` | `nav_msgs/Odometry` | Pose locale (EKF odom) |
| Souscrit | `/map` | `nav_msgs/OccupancyGrid` | Carte d'occupation |
| Souscrit | `/scan` | `sensor_msgs/LaserScan` | Obstacles temps réel |
| Publie | `/cmd_vel` | `geometry_msgs/Twist` | Commande de vitesse |
| Expose | `/follow_path` | `nav2_msgs/action/FollowPath` | Exécution d'un chemin |

---

## 5. Module faucon_control

### Nœud : control_4ws (CmdVelToJoints)

**Rôle** : convertit `/cmd_vel` en commandes de joints pour un robot 4 roues directrices (4WS) via `ros2_control`.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Souscrit | `/cmd_vel` | `geometry_msgs/Twist` | Commande de vitesse Nav2 |
| Publie | `/steer_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Angles de braquage |
| Publie | `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Vitesses angulaires roues |

**Paramètres ROS2** :

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `wheel_radius` | `0.25` m | Rayon des roues |
| `wheel_base` | `0.65` m | Empattement avant/arrière |
| `track_width` | `1.0` m | Voie (gauche/droite) |
| `max_steer` | `0.785` rad (45°) | Limite d'angle de braquage |
| `max_steer_rate` | `0.5` rad/s | Vitesse max de changement de braquage |

> Ces paramètres doivent être adaptés par profil de robot. Ne pas hardcoder dans le code source.

---

## 6. Module faucon_drone

Intègre un quadrotor PX4 X500 dans le même monde Gazebo que l'UGV.
Toute la logique PX4/NED est isolée dans `PX4X500Adapter` — le reste du stack ne voit que l'interface `/faucon/drone/*`.

### Architecture interne

```
Gazebo (x500_mono_cam)  ←GZ transport→  PX4 SITL (standalone)
                                              ↕  XRCE-DDS UDP:8888
                                         MicroXRCEAgent
                                              ↕  /fmu/*
                                         PX4X500Adapter
                                              ↕
                                         /faucon/drone/*
```

### Nœud : PX4X500Adapter

**Rôle** : traduit PX4 uORB (/fmu/*) ↔ interface stable Faucon (/faucon/drone/*). Gère la conversion de repère NED ↔ ENU et diffuse la transformée TF `odom → drone_base_link`.

#### Topics publiés

| Topic | Type | Fréquence | Description |
|-------|------|-----------|-------------|
| `/faucon/drone/odom` | `nav_msgs/Odometry` | 50 Hz | Position + vitesse, repère ENU local |
| `/faucon/drone/gps` | `sensor_msgs/NavSatFix` | ~5 Hz | Fix GPS WGS-84 |
| `/faucon/drone/imu` | `sensor_msgs/Imu` | ~250 Hz | Attitude + taux, body-ENU |
| `/faucon/drone/state` | `std_msgs/String` | ~5 Hz | `DISARMED` / `ARMED` / `OFFBOARD` / … |
| `/faucon/drone/camera/image` | `sensor_msgs/Image` | 10–30 Hz | Caméra RGB embarquée |

#### Topics consommés

| Topic | Type | Description |
|-------|------|-------------|
| `/faucon/drone/cmd_vel` | `geometry_msgs/Twist` | Vitesse body-ENU : x=avant, y=gauche, z=haut, ω_z=yaw CCW+ |
| `/faucon/drone/arm` | `std_msgs/Bool` | `true` = arm + offboard, `false` = disarm |

#### Convention de repère

| Axe | Body-ENU (Faucon) |
|-----|------------------|
| `linear.x` | avant |
| `linear.y` | gauche |
| `linear.z` | haut |
| `angular.z` | yaw, sens CCW positif (standard ROS) |

### Nœud : drone_takeoff_hover

**Rôle** : décollage automatique et maintien en vol stationnaire. Actif par défaut (`auto_takeoff:=true`). Désactivé automatiquement si `auto_trajectory:=true`.

| Phase | Description |
|-------|-------------|
| WAITING | Attend l'odomètrie + `start_delay` |
| PREARM | Diffuse `cmd_vel=0` pendant `prearm_setpoint_time`, puis arm + offboard |
| TAKEOFF | Monte à `target_altitude` en maintenant le XY de spawn |
| HOVER | Régule autour de la position atteinte (contrôleur P) |

### Nœud : drone_trajectory

**Rôle** : décollage autonome puis exécution d'une séquence de waypoints. Remplace `drone_takeoff_hover` quand `auto_trajectory:=true`.

| Phase | Description |
|-------|-------------|
| WAITING | Attend l'odomètrie + `start_delay` |
| PREARM | Idem drone_takeoff_hover |
| TAKEOFF | Monte à `takeoff_altitude` |
| TRAJECTORY | Visite chaque waypoint `(x, y, z)` ENU en ordre |
| HOVER | Maintient la dernière position atteinte |

#### Topic publié

| Topic | Type | Description |
|-------|------|-------------|
| `/faucon/drone/trajectory/status` | `std_msgs/String` | Phase courante : `WAITING` / `PREARM` / `TAKEOFF` / `WP_N/TOTAL` / `HOVER` |

#### Paramètres principaux

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `waypoints` | `[]` | Liste plate `[x0,y0,z0, x1,y1,z1, …]` ENU (m) |
| `waypoints_str` | `""` | Variante CSV `"x0,y0,z0,x1,y1,z1,…"` (utilisé par `trajectory_waypoints:=` en launch) |
| `takeoff_altitude` | `3.0` m | Altitude cible avant le premier waypoint |
| `waypoint_tolerance` | `0.5` m | Rayon d'arrivée 3D par waypoint |
| `cruise_speed` | `1.2` m/s | Vitesse horizontale max par segment |
| `return_home` | `false` | Ajoute un waypoint retour à la position de spawn |
| `start_delay` | `20.0` s | Délai avant armement (convergence PX4/EKF2) |

#### Exemple de lancement avec trajectoire

```bash
ros2 launch faucon_drone drone_sim.launch.py \
  auto_trajectory:=true \
  trajectory_waypoints:="7.0,2.0,3.0,9.0,2.0,3.0,9.0,-2.0,3.0,7.0,-2.0,3.0"
```

---

## 7. Module faucon_ihm

### Services réseau

| Service | Port | Protocole | Description |
|---------|------|-----------|-------------|
| rosbridge | `9090` | WebSocket | Bridge ROS2 ↔ JavaScript (roslibjs) |
| web_video_server | `8080` | HTTP | Flux vidéo `/camera/image` et `/camera/depth_image` |
| IHM React | `3000` | HTTP | Interface web de monitoring et de commande |

L'IHM consomme via rosbridge :
- `/mission/status` — état de la mission
- `/mission/path` — chemin affiché sur la carte
- `/rtabmap/cloud_map` — carte 3D (si activée)
- `/camera/image` — flux caméra (via web_video_server)

L'IHM publie via rosbridge :
- `/mission/load_path` — envoi d'une mission
- `/mission/command` — START / STOP / PAUSE / RESUME

---

## 8. TF frames

```
map
 └── odom                    (publié par ekf_filter_node_map)
      └── base_link           (publié par hardware/simulateur via tf)
           ├── lidar_frame
           ├── imu_link
           └── camera_link

odom
 └── drone_base_link          (publié par PX4X500Adapter — 50 Hz)
```

| Frame | Publié par | Description |
|-------|-----------|-------------|
| `map` | ekf_filter_node_map | Repère global ancré sur le datum GNSS |
| `odom` | hardware / Gazebo | Repère local UGV, dérive sur le long terme |
| `base_link` | hardware / Gazebo | Centre de l'UGV |
| capteurs UGV | hardware / URDF | Positions fixes des capteurs sur l'UGV |
| `drone_base_link` | PX4X500Adapter | Centre du drone, frame enfant de `odom` |

> `drone_base_link` partage la même frame parente `odom` que l'UGV — les deux agents sont
> ainsi dans le même repère local sans fusion GNSS supplémentaire.

---

## 9. Récapitulatif — ce que Faucon expose à l'opérateur

| Topic / Port | Type | Usage |
|-------------|------|-------|
| `/mission/status` | `std_msgs/String` (JSON) | État temps réel de la mission |
| `/mission/path` | `nav_msgs/Path` | Chemin de mission en cours |
| `/mission/load_path` | `std_msgs/String` (YAML) | **Entrée** : charger une mission |
| `/mission/command` | `std_msgs/String` | **Entrée** : START/STOP/PAUSE/RESUME |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | Carte 3D temps réel |
| `/rtabmap/map` | `nav_msgs/OccupancyGrid` | Carte 2D pour visualisation |
| `/cloud_calib_out` | `sensor_msgs/PointCloud2` | Nuage LiDAR filtré |
| `/ground_cloud` | `sensor_msgs/PointCloud2` | Plan sol détecté |
| `odometry/global` | `nav_msgs/Odometry` | Position absolue du robot |
| `/gnss/datum` | `sensor_msgs/NavSatFix` | Origine GPS de la session |
| `:9090` WebSocket | rosbridge | Accès complet aux topics depuis le web |
| `:8080` HTTP | web_video_server | Flux vidéo caméra |
| `:3000` HTTP | IHM React | Interface de monitoring |
| `/faucon/drone/odom` | `nav_msgs/Odometry` | Position + vitesse du drone (ENU) |
| `/faucon/drone/state` | `std_msgs/String` | État PX4 : DISARMED / ARMED / OFFBOARD |
| `/faucon/drone/trajectory/status` | `std_msgs/String` | Phase trajectoire : WP_N/TOTAL / HOVER |
| `/faucon/drone/camera/image` | `sensor_msgs/Image` | Caméra embarquée drone |
| `/faucon/drone/cmd_vel` | `geometry_msgs/Twist` | **Entrée** : commande de vitesse body-ENU |
| `/faucon/drone/arm` | `std_msgs/Bool` | **Entrée** : arm (`true`) / disarm (`false`) |
