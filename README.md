## Faucon description repertoire

# Virtual Maize Field – Demo de lancement (ROS 2 + Gazebo)

Ce dépôt lance un robot URDF dans un monde Gazebo (champ de maïs virtuel), avec ponts ROS ↔ Gazebo et visualisation RViz.

![Aperçu du résultat](docs/base_robot.png)

## Prérequis

- ROS 2 (Humble ou +)
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`
- Packages du projet :
  - `base_desc` (contient `description/robot.urdf.xacro`)
  - `virtual_maize_field` (contient le launch `simulation.launch.py` et les ressources du monde)

Assure-toi d’avoir compilé et sourcé l’espace :
```bash
colcon build --packages-up-to base_desc virtual_maize_field
source install/setup.bash
