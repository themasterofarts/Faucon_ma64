# Trajectory Generator module

`trajectory_generator.py` genere une trajectoire YAML a partir de waypoints YAML.

Le script est independant de ROS/Nav2:
- pas de publication de topics
- pas de conversion de coordonnees
- il fait uniquement du `read -> generate -> write`

## 1) Ce que fait l'outil

Deux modes:

1. `legacy` (par defaut): arrondi geometrique local `ligne + arc`
2. `dubins`: pose-a-pose avec yaw d'entree (`--use-input-yaw`)

## 2) Format YAML attendu

Exemple entree:

```yaml
waypoints:
  - latitude: 43.8999785
    longitude: 3.1999708
    yaw: 0.02617
  - latitude: 43.9000184
    longitude: 3.1999718
    yaw: 1.55216
```

Par defaut:
- liste d'entree: `waypoints`
- cles points: `latitude`, `longitude`
- cle yaw: `yaw`

Tu peux changer ces noms via options CLI.

## 3) Parametres importants

- `--step`: pas d'echantillonnage
- `--turn-radius`: rayon de virage
- `--use-input-yaw`: active le mode Dubins pose-a-pose

Important:
- `step` et `turn-radius` doivent etre dans la meme unite que les coordonnees.
- Si coordonnees GNSS (degres), utiliser de petites valeurs (ex `1e-6`, `3e-6`).
- Si coordonnees locales en metres, utiliser des valeurs en metres (ex `0.1`, `5`).

## 4) Exemples d'execution

### Legacy

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_legacy.yaml \
  --step 0.01 \
  --turn-radius 5
```

### Dubins (avec yaw d'entree)

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_dubins.yaml \
  --step 0.01 \
  --turn-radius 5 \
  --use-input-yaw
```

### Avec visualisation PNG

```bash
python3 trajectory_generator.py \
  --input gps_waypoints_sent.yaml \
  --output path_dubins.yaml \
  --step 0.01 \
  --turn-radius 5 \
  --use-input-yaw \
  --visualize
```

Le plot sera ecrit par defaut a cote du YAML de sortie:
- `path_dubins.png`

Tu peux forcer le chemin:

```bash
--plot-output trajectory_preview.png
```

## 5) Sortie YAML

Par defaut:
- liste sortie: `trajectory`
- cles points: memes cles que l'entree (`latitude`, `longitude` par defaut)
- yaw inclus (desactive avec `--no-yaw`)

