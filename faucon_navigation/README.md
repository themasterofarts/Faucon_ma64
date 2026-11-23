# faucon_navigation

Brique de navigation autonome du robot Faucon. Gère la conversion de waypoints GPS en trajectoires ENU, l'exécution via Nav2, et le cycle de vie complet d'une mission depuis l'IHM.

## Architecture

```
IHM (React)
  │  /mission/load_path  (YAML)
  │  /mission/command    (START | STOP | PAUSE | RESUME)
  ▼
┌─────────────────────────────────────┐
│          mission_manager            │
│  ┌──────────────────────────────┐   │
│  │        mission_core          │   │  ← logique pure Python (testable sans ROS)
│  │  State, validate_waypoints   │   │    State machine, validation YAML,
│  │  nearest_wp_index            │   │    géométrie des waypoints
│  └──────────────────────────────┘   │
│                                     │
│  /gnss/datum ──────────────────────►│ attend le datum avant toute conversion
│  odometry/global ──────────────────►│ tracking de la progression
│                                     │
│  ──► /fromLLArray (service)         │ convertit GPS → ENU
│  ──► /follow_path (action Nav2)     │ exécute la trajectoire
│                                     │
│  /mission/status ◄──────────────────│ JSON 5 Hz (état, progression, erreurs)
│  /mission/path   ◄──────────────────│ nav_msgs/Path (TRANSIENT_LOCAL, pour RViz2)
└─────────────────────────────────────┘
```

### Nodes

| Node | Rôle |
|------|------|
| `mission_manager` | Gère le cycle de vie d'une mission (chargement YAML → conversion GPS → exécution Nav2 → statut IHM) |
| `follower` | Envoi one-shot d'un fichier YAML de waypoints vers Nav2 (usage standalone / debug) |
| `gps_waypoint_logger` | Enregistre des waypoints GPS en temps réel dans un fichier YAML |

### Cycle de vie d'une mission

```
IDLE ──(load_path)──► LOADING ──(conversion OK)──► READY
                                                      │
                                               (START)│
                                                      ▼
                                                  RUNNING ──(succès)──► COMPLETED ──(3s)──► IDLE
                                                      │
                                          (PAUSE)     │     (STOP)
                                              ▼        └──────────► ABORTED ──► IDLE
                                           PAUSED
                                              │
                                         (RESUME) — reprend depuis le WP courant
```

### Topics & services

| Interface | Type | Direction |
|-----------|------|-----------|
| `/mission/load_path` | `String` (YAML) | entrée |
| `/mission/command` | `String` | entrée |
| `/gnss/datum` | `NavSatFix` (latch) | entrée |
| `odometry/global` | `Odometry` | entrée |
| `/mission/status` | `String` (JSON) | sortie 5 Hz |
| `/mission/path` | `Path` (latch) | sortie |
| `/fromLLArray` | service `robot_localization` | appel |
| `/follow_path` | action `nav2_msgs` | appel |

### Format YAML des waypoints

```yaml
waypoints:
  - latitude: 43.8999
    longitude: 3.1999
    yaw: 0.0          # optionnel, radians
  - latitude: 43.9001
    longitude: 3.2002
```

### Démarrage

```bash
# Navigation complète (Nav2 + localisation + mission_manager)
ros2 launch faucon_navigation faucon_nav.launch.py use_sim_time:=true

# Follower standalone (envoi direct d'un YAML)
ros2 run faucon_navigation follower.py --ros-args -p yaml_path:=/chemin/vers/path.yaml
```
