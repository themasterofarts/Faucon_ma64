#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import yaml
import os
import sys
import time
import tkinter as tk
from tkinter import messagebox
import math
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion

class GpsGuiLogger(tk.Tk, Node):
    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_logger')
        self.title("GPS Waypoint Logger")

        self.logging_file_path = logging_file_path
        self.waypoints_list = []
        
        # Timestamps pour vérifier la fraîcheur des données
        self.last_gps_time = 0.0
        self.last_imu_time = 0.0
        self.data_timeout = 2.0  # secondes

        # --- UI Setup ---
        self.status_label = tk.Label(self, text="Status: Waiting for sensors...", fg="orange", font=('Arial', 10, 'bold'))
        self.status_label.pack(pady=5)

        self.gps_pose_textbox = tk.Label(self, text="Waiting for GPS & IMU...", width=50, height=4, relief="sunken")
        self.gps_pose_textbox.pack(pady=10)

        self.log_gps_wp_button = tk.Button(self, text="Log Waypoint", command=self.log_waypoint,
                                           bg="#2ecc71", fg="white", font=('Arial', 11, 'bold'))
        self.log_gps_wp_button.pack(pady=10)

        # --- Subscriptions ---
        self.gps_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.last_gps_position = NavSatFix()
        self.last_heading = 0.0

        # Timer pour mettre à jour l'UI régulièrement (vérifier le timeout)
        self.ui_update_timer = self.create_timer(0.5, self.update_ui)

    def euler_from_quaternion(self, q: Quaternion):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.last_gps_position = msg
            self.last_gps_time = time.time()
            self.update_ui()

    def imu_callback(self, msg: Imu):
        _, _, self.last_heading = self.euler_from_quaternion(msg.orientation)
        self.last_imu_time = time.time()
        self.update_ui()

    def is_data_valid(self):
        """Vérifie si les données sont récentes,genre moins de data_timeout secondes"""
        current_time = time.time()
        gps_valid = (current_time - self.last_gps_time) < self.data_timeout
        imu_valid = (current_time - self.last_imu_time) < self.data_timeout
        return gps_valid and imu_valid

    def update_ui(self):
        current_time = time.time()
        gps_valid = (current_time - self.last_gps_time) < self.data_timeout
        imu_valid = (current_time - self.last_imu_time) < self.data_timeout
        
        if gps_valid and imu_valid:
            self.status_label.config(text="Status: ALL SYSTEMS READY", fg="green")
            self.gps_pose_textbox.config(
                text=f"LAT: {self.last_gps_position.latitude:.7f}\nLON: {self.last_gps_position.longitude:.7f}\nYAW: {self.last_heading:.3f} rad")
        elif gps_valid and not imu_valid:
            self.status_label.config(text="Status: IMU TIMEOUT ", fg="orange")
            self.gps_pose_textbox.config(text="GPS OK, waiting for IMU...")
        elif not gps_valid and imu_valid:
            self.status_label.config(text="Status: GPS TIMEOUT ", fg="orange")
            self.gps_pose_textbox.config(text="IMU OK, waiting for GPS fix...")
        else:
            self.status_label.config(text="Status: NO DATA (timeout) ", fg="red")
            self.gps_pose_textbox.config(text="Waiting for GPS & IMU...")

    def log_waypoint(self):
        # Vérification que les données sont valides ET récentes
        if not self.is_data_valid():
            current_time = time.time()
            missing = []
            if (current_time - self.last_gps_time) >= self.data_timeout:
                missing.append("GPS (timeout)")
            if (current_time - self.last_imu_time) >= self.data_timeout:
                missing.append("IMU (timeout)")
            messagebox.showwarning("Données invalides", 
                f"Données trop anciennes ou manquantes :\n{', '.join(missing)}\n\nAssurez-vous que les capteurs publient des données.")
            return

        data = {
            "latitude": float(self.last_gps_position.latitude),
            "longitude": float(self.last_gps_position.longitude),
            "yaw": float(self.last_heading)
        }
        
        self.waypoints_list.append(data)

        try:
            with open(self.logging_file_path, 'w') as yaml_file:
                yaml.dump({"waypoints": self.waypoints_list}, yaml_file, default_flow_style=False)
            
            messagebox.showinfo("Succès ✓", 
                f"Waypoint #{len(self.waypoints_list)} enregistré !\n\nFichier : {self.logging_file_path}")
        except Exception as e:
            messagebox.showerror("Erreur Fichier", 
                f"Impossible d'écrire dans le fichier :\n{str(e)}")
def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        # Chemin personnalisé fourni en argument
        yaml_file_path = sys.argv[1]
        print(f"Using custom path: {yaml_file_path}")
    else:
        yaml_file_path = os.path.expanduser('~/Faucon_ma64/faucon_navigation/config/gps_waypoints.yaml')
        print(f"Waypoints will be saved to: {yaml_file_path}")

    # Créer le répertoire config s'il n'existe pas
    os.makedirs(os.path.dirname(yaml_file_path), exist_ok=True)

    node = GpsGuiLogger(yaml_file_path)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update()
    except (KeyboardInterrupt, tk.TclError):
        print("\nArrêt du logger GPS...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()