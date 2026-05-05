# faucon_localisation

Brique de localisation du robot Faucon. Fusionne les données GNSS, IMU et odométrie pour produire deux estimées de pose : locale (frame `odom`) et globale (frame `map`).

## Architecture

```
/gnss/fix  ──────────────────────────────────────────────────────────┐
                                                                      │
                         ┌─────────────────┐                         │
                         │  datum_manager  │◄────────────────────────┘
                         │                 │   capte le 1er fix valide
                         └────────┬────────┘
                                  │ SetDatum (service /datum)
                                  ▼
         ┌────────────────────────────────────────────┐
         │          navsat_transform_node              │
         │  Convertit GNSS → ENU (frame map)           │
         │  Service /fromLLArray (GPS → coordonnées ENU)│
         └──────────────┬───────────────┬─────────────┘
                        │               │
               /gps/filtered     /odometry/gps
                        │               │
         ┌──────────────▼───┐   ┌───────▼──────────────┐
         │  ekf_filter_node  │   │  ekf_filter_node      │
         │  _map (global)    │   │  _odom (local)        │
         │  frame: map       │   │  frame: odom          │
         └──────────────┬───┘   └───────┬──────────────┘
                        │               │
               odometry/global   odometry/local
```

### Nodes

| Node | Rôle |
|------|------|
| `datum_manager` | Capture le premier fix GNSS valide, appelle le service `/datum` pour ancrer `navsat_transform`, publie `/gnss/datum` (TRANSIENT_LOCAL) |
| `navsat_transform_node` | Convertit les coordonnées GPS en ENU, expose `/fromLLArray` |
| `ekf_filter_node_odom` | EKF local — fusionne IMU + encodeurs → `odometry/local` dans `odom` |
| `ekf_filter_node_map` | EKF global — fusionne odométrie locale + GNSS → `odometry/global` dans `map` |

### Topics clés

| Topic | Type | Description |
|-------|------|-------------|
| `/gnss/fix` | `NavSatFix` | Données brutes GNSS |
| `/gnss/datum` | `NavSatFix` (latch) | Origine de la frame locale — consommé par la navigation |
| `odometry/local` | `Odometry` | Pose dans `odom` (court terme, sans dérive) |
| `odometry/global` | `Odometry` | Pose dans `map` (long terme, ancrée GNSS) |

### Démarrage

```bash
ros2 launch faucon_localisation dual_ekf_navsat.launch.py
```

Le datum est configuré dynamiquement au premier fix : `wait_for_datum: true` dans `dual_ekf_navsat_params.yaml`.
