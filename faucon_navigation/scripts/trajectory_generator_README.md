# Trajectory Generator module

`trajectory_generator.py` genere une trajectoire YAML a partir de waypoints YAML.

Le script est independant de ROS/Nav2:
- pas de publication de topics
- conversion GNSS->local->GNSS active par defaut
- `read -> generate -> write`

## 1) Mode unique

Le script fonctionne uniquement en mode rangs:
- segments 1->2, 3->4, 5->6... en ligne droite
- segments 2->3, 4->5... en virages Dubins

Format attendu:
- points ordonnes comme `[entree_rang1, sortie_rang1, entree_rang2, sortie_rang2, ...]`
- nombre pair de points

Important:
- ce mode ne lit pas les `yaw` d'entree
- `step` et `turn-radius` sont en metres (quand GNSS->local est actif, defaut)

## 2) Format YAML attendu

Exemple:

```yaml
waypoints:
  - latitude: 43.8999785
    longitude: 3.1999708
  - latitude: 43.9000184
    longitude: 3.1999718
```

Par defaut:
- liste entree: `waypoints`
- cles: `latitude`, `longitude`

## 3) Parametres CLI

- `--step`: pas d'echantillonnage (m)
- `--turn-radius`: rayon de virage (m)
- `--origin-lat`, `--origin-lon`: origine locale optionnelle (sinon 1er waypoint)
- `--no-gnss-to-local`: desactive la conversion GNSS->local->GNSS
- `--earth-radius-m`: rayon terrestre pour la conversion (defaut `6378137.0`)
- `--no-yaw`: ne pas ecrire le yaw de sortie
- `--visualize`: genere un PNG
- `--plot-output`: chemin PNG

## 4) Exemples

### Generation trajectoire (recommande)

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_turns.yaml \
  --step 0.1 \
  --turn-radius 1.0
```

### Avec visualisation

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_turns.yaml \
  --step 0.1 \
  --turn-radius 1.0 \
  --visualize
```

### Avec origine locale explicite

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_turns.yaml \
  --step 0.1 \
  --turn-radius 1.0 \
  --origin-lat 43.8999785 \
  --origin-lon 3.1999708
```

### Cles YAML personnalisees

```bash
python3 trajectory_generator.py \
  --input my_points.yaml \
  --output my_traj.yaml \
  --step 0.1 \
  --turn-radius 1.0 \
  --input-list-key points \
  --output-list-key path \
  --x-key lat \
  --y-key lon \
  --yaw-key heading
```

## 5) Sortie YAML

Par defaut:
- liste sortie: `trajectory`
- cles: `latitude`, `longitude`
- `yaw` inclus (sauf `--no-yaw`)
