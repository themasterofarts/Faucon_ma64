# ros2_ihm — Ground Control Interface

IHM web modulaire pour ROS2. Accessible depuis n'importe quel navigateur sur le LAN.

## Fonctionnalités V1

| Widget | Topic ROS2 | Description |
|--------|-----------|-------------|
| 🗺 Carte GPS | `/gnss/fix` | OpenStreetMap + trace de trajectoire |
| ✈ Cockpit IMU | `/imu/data` | Horizon artificiel roll/pitch/yaw |
| 📷 Caméra | `web_video_server` | Flux MJPEG configurable |
| 🕹 Contrôle | `/cmd_vel` | Joystick souris/touch + clavier ZQSD |
| 📡 Télémétrie | `/odom` | Vitesse, uptime, statut Nav2 |

---

## Installation

### 1. Dépendances ROS2

```bash
sudo apt install \
  ros-$ROS_DISTRO-rosbridge-suite \
  ros-$ROS_DISTRO-web-video-server
```

### 2. Cloner dans votre workspace

```bash
cd ~/ros2_ws/src
# Copier le dossier ros2_ihm ici
```

### 3. Builder l'IHM React (une seule fois)

```bash
cd ~/ros2_ws/src/ros2_ihm/ihm
npm install
npm run build          # génère ihm/dist/
```

### 4. Builder le package ROS2

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_ihm
source install/setup.bash
```

---

## Utilisation

### Lancement tout-en-un

```bash
ros2 launch ros2_ihm ihm_launch.py
```

L'IHM est accessible sur **http://\<IP-robot\>:3000**

### Options

```bash
# Changer les ports
ros2 launch ros2_ihm ihm_launch.py rosbridge_port:=9090 video_port:=8080 ihm_port:=3000

# Développement (hot-reload)
cd ihm && npm run dev
# → http://localhost:5173
```

---

## Développement de l'IHM

En mode dev, Vite sert l'IHM avec hot-reload :

```bash
cd ~/ros2_ws/src/ros2_ihm/ihm
npm run dev
```

Après modifications, rebuilder pour la production :

```bash
npm run build:ros
# Puis recolcon si nécessaire (CMakeLists installe dist/)
colcon build --packages-select ros2_ihm
```

---

## Architecture du package

```
ros2_ihm/
├── package.xml                  # Manifest ROS2
├── CMakeLists.txt               # Build + install
├── README.md
│
├── launch/
│   ├── ihm_launch.py            # Launch file principal
│   └── ihm_server.py            # Serveur HTTP Python (SPA)
│
├── ros2_ihm/                    # Nœuds Python (extensible)
│   └── __init__.py
│
├── ihm/                         # Application React
│   ├── package.json
│   ├── vite.config.js
│   ├── index.html
│   ├── src/
│   │   ├── main.jsx
│   │   └── App.jsx              # Dashboard principal
│   └── dist/                    # Build production (gitignore)
│
└── resource/
    └── ros2_ihm                 # Marker ament
```

---

## Ajouter un widget

1. Créer un composant dans `ihm/src/widgets/MonWidget.jsx`
2. L'importer dans `App.jsx`
3. Ajouter un panel dans la grille dashboard
4. `npm run build:ros` + `colcon build`

---

## Topics ROS2 utilisés

```
Subscriptions:
  /gnss/fix          sensor_msgs/NavSatFix
  /imu/data          sensor_msgs/Imu
  /odom              nav_msgs/Odometry

Publications:
  /cmd_vel           geometry_msgs/Twist
```

## Licence

MIT
