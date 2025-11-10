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

### 🧩 Architecture du projet
```bash 
Faucon_ma64/
├── Faucon_Orchestration/ # Coordination robot + drone via Behavior Trees
├── faucon/ # Scripts principaux, lancement global
├── faucon_base_desc/ # Description URDF/Xacro du robot
├── faucon_control/ # Contrôleurs 4WS, plugins ROS2 Control
├── faucon_drone/ # Intégration et simulation du drone PX4
├── faucon_localisation/ # Fusion GPS, IMU, odométrie
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

sudo chmod +x ./faucon/launch_base.sh

./faucon/launch_base.sh
```
#### Lancement mini robot
```bash
./faucon/launch_base.sh  use_mini=false
```
---

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

### Remerciements

Un grand merci à tous les membres de la communauté MA64 Robotics pour leur implication, leurs idées et leur passion pour la robotique.
Ensemble, faisons de Faucon un projet de référence en robotique agricole open source ! 🚜🤖
