# 🦅 faucon_ihm : Ground Control Interface

> Interface web temps réel pour ROS2. Accessible depuis n'importe quel navigateur sur le LAN - PC, tablette, smartphone.

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Jazzy-blue)
![React](https://img.shields.io/badge/React-18-61dafb)
![Leaflet](https://img.shields.io/badge/Map-OpenStreetMap-green)

---

## Widgets disponibles (V1)

| Widget | Topic ROS2 | Description |
|--------|------------|-------------|
| 🗺 Carte GPS | `/gnss/fix` | Carte OpenStreetMap + trace de trajectoire temps réel |
| ✈ Cockpit IMU | `/imu/data` | Horizon artificiel — roll / pitch / cap |
| 📷 Caméra | `web_video_server :8080` | Flux MJPEG configurable via URL |
| 🕹 Contrôle manuel | `/cmd_vel` | Joystick souris/touch + clavier ZQSD/flèches |
| 📡 Télémétrie | `/odom` | Vitesse, uptime, statut Nav2 |

---

## Prérequis

- ROS2 (Humble, Iron ou Jazzy)
- Node.js ≥ 18 (`node --version`)
- npm ≥ 9

---

## Installation

### 1. Dépendances ROS2

```bash
sudo apt install \
  ros-$ROS_DISTRO-rosbridge-suite \
  ros-$ROS_DISTRO-web-video-server
```

### 2. Builder l'IHM React _(une seule fois, ou après modification)_

```bash
cd ~/Faucon_ma64/faucon_ihm/ihm
npm install
npm run build        # génère ihm/dist/
```

### 3. Builder le package ROS2

```bash
colcon build --packages-select faucon_ihm
source install/setup.bash
```

---

## Lancement

### Tout-en-un (recommandé)

```bash
ros2 launch faucon_ihm ihm_launch.py
```

Le terminal affiche les URLs d'accès :

```
╔══════════════════════════════════════════════╗
║        ROS2 IHM — GROUND CONTROL             ║
╠══════════════════════════════════════════════╣
║  rosbridge   ws://0.0.0.0:9090               ║
║  camera      http://0.0.0.0:8080             ║
║  IHM         http://0.0.0.0:3000             ║
╚══════════════════════════════════════════════╝
```

Ouvrez **`http://<IP-robot>:3000`** dans votre navigateur.

> Pour connaître l'IP de votre machine ROS2 : `hostname -I`

### Avec ports personnalisés

```bash
ros2 launch faucon_ihm ihm_launch.py \
  rosbridge_port:=9090 \
  video_port:=8080 \
  ihm_port:=3000
```

---

## Configuration de l'IHM

Au premier lancement, cliquez sur **⚙ CONFIG** (en haut à droite) et renseignez :

| Champ | Valeur par défaut | Description |
|-------|------------------|-------------|
| ROSBRIDGE HOST | `ws://localhost:9090` | Adresse WebSocket de rosbridge |
| VIDEO SERVER | `http://localhost:8080/stream?topic=/camera/image_raw` | URL du flux caméra MJPEG |
| GPS TOPIC | `/gnss/fix` | Topic NavSatFix |
| IMU TOPIC | `/imu/data` | Topic Imu |
| CMD_VEL TOPIC | `/cmd_vel` | Topic de commande vitesse |

> Si l'IHM est sur une machine différente du robot, remplacez `localhost` par l'IP du robot.

---

## Contrôle clavier

| Touche | Action |
|--------|--------|
| `Z` ou `W` ou `↑` | Avancer |
| `S` ou `↓` | Reculer |
| `Q` ou `A` ou `←` | Tourner à gauche |
| `D` ou `→` | Tourner à droite |
| Bouton **EMERGENCY STOP** | Stoppe immédiatement le robot (`cmd_vel` = 0) |

---

## Mode développement (hot-reload)

Pour modifier l'IHM avec rechargement automatique :

```bash
cd /faucon_ihm/ihm
npm run dev
# → http://localhost:5173  (accessible aussi sur le LAN)
```

Après vos modifications, builder pour la production :

```bash
npm run build
# Pas besoin de recolcon — le serveur HTTP sert dist/ directement
```

Si vous modifiez `CMakeLists.txt` ou `launch/` :

```bash
colcon build --packages-select faucon_ihm
source install/setup.bash
```

---

## Architecture du package

```
faucon_ihm/
│
├── package.xml                  ← Manifest ROS2
├── CMakeLists.txt               ← Build & install rules
├── README.md                    ← Ce fichier
│
├── launch/
│   ├── ihm_launch.py            ← Lance rosbridge + web_video_server + IHM
│   └── ihm_server.py            ← Serveur HTTP Python pour le build React
│
├── faucon_ihm/                  ← Module Python (futurs nœuds ROS2)
│   └── __init__.py
│
├── ihm/                         ← Application React (Vite)
│   ├── package.json
│   ├── vite.config.js
│   ├── index.html
│   └── src/
│       ├── main.jsx             ← Point d'entrée React
│       └── App.jsx              ← Dashboard complet (widgets + rosbridge hook)
│
└── resource/
    └── faucon_ihm               ← Marker ament (ne pas supprimer)
```

---

## Ajouter un widget

1. Créer votre composant dans `ihm/src/App.jsx` (ou dans un fichier séparé `ihm/src/widgets/`)
2. L'ajouter dans la grille dashboard (`<div className="dashboard">`)
3. `npm run build` dans `ihm/`

Les widgets reçoivent les props `subscribe` et/ou `publish` pour interagir avec ROS2 :

```jsx
function MonWidget({ subscribe, publish }) {
  useEffect(() => {
    const unsub = subscribe("/mon/topic", "std_msgs/String", (msg) => {
      console.log(msg.data);
    });
    return unsub; // cleanup automatique à la destruction du composant
  }, [subscribe]);
}
```

---

## Topics ROS2

```
Subscriptions (lecture) :
  /gnss/fix     sensor_msgs/NavSatFix   — position GPS
  /imu/data     sensor_msgs/Imu         — orientation IMU
  /odom         nav_msgs/Odometry       — odométrie

Publications (écriture) :
  /cmd_vel      geometry_msgs/Twist     — commande vitesse
```

---

## Dépannage

**L'IHM s'affiche mais "DISCONNECTED" reste affiché**
→ Vérifiez que rosbridge tourne : `ros2 node list | grep rosbridge`
→ Vérifiez l'adresse dans ⚙ CONFIG (IP du robot, pas localhost si accès distant)

**La carte GPS ne se centre pas**
→ Vérifiez que le topic publie : `ros2 topic echo --once /gnss/fix`
→ Vérifiez le nom du topic dans ⚙ CONFIG

**Pas d'image caméra**
→ Vérifiez web_video_server : `ros2 node list | grep video`
→ Testez l'URL directement dans le navigateur : `http://<IP>:8080/stream?topic=/camera/image_raw`

**Erreur `colcon build` — package introuvable**
→ Vérifiez que le dossier s'appelle bien `faucon_ihm` (= nom dans `package.xml`)

---

## Licence

MIT — Libre d'utilisation, modification et distribution.
