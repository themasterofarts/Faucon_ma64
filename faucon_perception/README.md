# faucon_perception

Brique de perception du robot Faucon : filtrage du nuage LiDAR et cartographie 3D avec RTAB-Map.


## Architecture

```
Gazebo /cloud (PointCloud2)
        │
        ▼
 lidar_calibrator          ← filtre le sol par RANSAC
  /cloud_calib_out         ← nuage propre (obstacles seuls)
  /ground_cloud            ← sol extrait (debug)
        │
        ├──────────────────────────────────────────┐
        ▼                                          ▼
 mapping_3d.launch.py              mapping_rgbd.launch.py
 (LiDAR seul)                      (LiDAR + caméra RGBD)
 rtabmap.yaml                      rtabmap_rgbd.yaml
 Reg/Strategy = ICP                Reg/Strategy = visual + ICP
        │                                          │
        └──────────────┬───────────────────────────┘
                       ▼
              /rtabmap/cloud_map   → nuage 3D
              /rtabmap/map         → OccupancyGrid 2D
```


## Nœuds

### `lidar_calibrator`

Filtre le sol du nuage LiDAR par segmentation RANSAC.

| Topic | Type | Direction |
|---|---|---|
| `/cloud` | `PointCloud2` | entrant (Gazebo bridge) |
| `/cloud_calib_out` | `PointCloud2` | sortant (obstacles) |
| `/ground_cloud` | `PointCloud2` | sortant (sol, debug) |


## Modes de cartographie

### Mode 1 - LiDAR seul

```bash
ros2 launch faucon_perception mapping_3d.launch.py
```

| Paramètre | Valeur |
|---|---|
| Config | `config/rtabmap.yaml` |
| Enregistrement | ICP pur (`Reg/Strategy=1`) |
| Source grille | `/cloud_calib_out` |
| Portée | 9 m |
| DB | `~/.ros/faucon_map.db` |

Adapté à une navigation à grande vitesse ou en l'absence de caméra fonctionnelle.

---

### Mode 2 - LiDAR + caméra RGBD (fusion)

```bash
ros2 launch faucon_perception mapping_rgbd.launch.py
```

| Paramètre | Valeur |
|---|---|
| Config | `config/rtabmap_rgbd.yaml` |
| Enregistrement | Visual + ICP (`Reg/Strategy=2`) |
| Sources | `/cloud_calib_out` + `/camera/image` + `/camera/depth_image` |
| Portée grille | 9 m (LiDAR) |
| DB | `~/.ros/faucon_map_rgbd.db` |

La caméra assure la **fermeture de boucle visuelle** (reconnaissance de lieu), le LiDAR assure l'**alignement ICP** précis de la carte.

#### Options communes

| Argument | Défaut | Description |
|---|---|---|
| `fresh_start` | `true` | Efface la DB au démarrage |
| `open_viz` | `false` | Ouvre `rtabmap_viz` |
| `database_path` | `~/.ros/faucon_map*.db` | Chemin de la base |
| `use_sim_time` | `true` | Temps simulé Gazebo |

```bash
# reprendre une session existante
ros2 launch faucon_perception mapping_rgbd.launch.py fresh_start:=false

# ouvrir le visualiseur RTAB-Map
ros2 launch faucon_perception mapping_rgbd.launch.py open_viz:=true
```

---

## Prérequis

Les deux modes nécessitent que la simulation et la navigation soient déjà lancées :

```bash
./faucon/launch_full.sh
# puis dans un autre terminal :
ros2 launch faucon_perception mapping_rgbd.launch.py
```
