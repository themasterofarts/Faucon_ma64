#!/usr/bin/env python3
"""
GNSS Waypoint Logger
Service ROS2 pour enregistrer les waypoints GNSS dans un fichier YAML
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Trigger
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def euler_from_quaternion(quaternion):
    """
    Convertit un quaternion en angles d'Euler (roll, pitch, yaw)
    quaternion: sensor_msgs/Quaternion
    retourne: (roll, pitch, yaw) en radians
    """
    import math
    
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    
    # Roll (rotation autour de l'axe x)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (rotation autour de l'axe y)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (rotation autour de l'axe z)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class GnssWaypointLogger(Node):
    """
    Nœud ROS2 qui enregistre les waypoints GNSS  dans un fichier YAML

    """

    def __init__(self):
        super().__init__('gnss_waypoint_logger')
        
        # Déclaration du paramètre pour le chemin du fichier
        self.declare_parameter('waypoints_file', '')
        waypoints_file = self.get_parameter('waypoints_file').value
        
        # Si pas de fichier spécifié, utiliser celui par défaut du package
        if not waypoints_file:
            package_share = get_package_share_directory('faucon_navigation')
            waypoints_file = os.path.join(package_share, 'config', 'waypoints_gnss.yaml')
            self.get_logger().info(f"Utilisation du fichier par défaut: {waypoints_file}")
        
        self.waypoint_file = waypoints_file
        
        # Variables pour stocker les dernières données
        self.last_gps_position = None
        self.last_heading = 0.0
        
        # Souscription au topic GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Souscription au topic IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Service pour enregistrer un waypoint
        self.log_service = self.create_service(
            Trigger,
            '/log_waypoint',
            self.log_waypoint_callback
        )
        
        self.get_logger().info('GNSS Waypoint Logger démarré')
        self.get_logger().info(f'Fichier de sortie: {self.waypoint_file}')
        self.get_logger().info('Appelez le service /log_waypoint pour enregistrer un point')

    def gps_callback(self, msg: NavSatFix):
        """Callback pour stocker la dernière position GPS"""
        self.last_gps_position = msg
        self.get_logger().debug(f'GNSS reçu: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')

    def imu_callback(self, msg: Imu):
        """Callback pour stocker le dernier cap (orientation)"""
        # Convertir quaternion en angles d'Euler
        _, _, self.last_heading = euler_from_quaternion(msg.orientation)
        self.get_logger().debug(f'IMU reçu: yaw={self.last_heading:.2f} rad')

    def log_waypoint_callback(self, request, response):
        """
        Service callback pour enregistrer un waypoint
        """
        # Vérifier qu'on a reçu des données GPS
        if self.last_gps_position is None:
            response.success = False
            response.message = 'Aucune donnée GNSS reçue pour le moment'
            self.get_logger().warn(response.message)
            return response
        
        # Lire les waypoints existants
        try:
            if os.path.exists(self.waypoint_file):
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f) or {"waypoints": []}
            else:
                data = {"waypoints": []}
        except Exception as e:
            response.success = False
            response.message = f'Erreur lecture fichier: {str(e)}'
            self.get_logger().error(response.message)
            return response
        
        new_waypoint = {
            'lat': float(self.last_gps_position.latitude),
            'lon': float(self.last_gps_position.longitude),
            'yaw': float(self.last_heading)
        }
        
        # Ajouter à la liste
        data['waypoints'].append(new_waypoint)
        
        # Sauvegarder dans le fichier
        try:
            # Créer le dossier si nécessaire
            os.makedirs(os.path.dirname(self.waypoint_file), exist_ok=True)
            
            with open(self.waypoint_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            response.success = True
            response.message = (
                f'Waypoint enregistré: '
                f'Lat={new_waypoint["lat"]:.6f}, '
                f'Lon={new_waypoint["lon"]:.6f}, '
                f'Yaw={new_waypoint["yaw"]:.2f} rad'
            )
            self.get_logger().info(response.message)
            self.get_logger().info(f'Total waypoints: {len(data["waypoints"])}')
            
        except Exception as e:
            response.success = False
            response.message = f'Erreur écriture fichier: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = GnssWaypointLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
