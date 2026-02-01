#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import yaml
import os
import sys
import tkinter as tk
from tkinter import messagebox
import math
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion



class GpsGuiLogger(tk.Tk, Node):
    """
    ROS2 node to log GPS waypoints to a file
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_logger')
        self.title("GPS Waypoint Logger")

        self.logging_file_path = logging_file_path
        
        # Liste pour stocker les waypoints en mémoire
        self.waypoints_list = []

        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack()
        self.gps_pose_textbox = tk.Label(self, text="", width=45)
        self.gps_pose_textbox.pack()

        self.log_gps_wp_button = tk.Button(self, text="Log GPS Waypoint",
                                           command=self.log_waypoint)
        self.log_gps_wp_button.pack()

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.gps_callback,
            1
        )
        self.last_gps_position = NavSatFix()

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            1
        )
        self.last_heading = 0.0

    def euler_from_quaternion(self, q: Quaternion):
        """
        Convert a quaternion into euler angles
        taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg
        self.updateTextBox()

    def imu_callback(self, msg: Imu):
        """
        Callback to store the last heading
        """
        _, _, self.last_heading = self.euler_from_quaternion(msg.orientation)
        self.updateTextBox()

    def updateTextBox(self):
        """
        Function to update the GUI with the last coordinates
        """
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, yaw: {self.last_heading:.2f} rad")

    def log_waypoint(self):
        """
        Function to save a new waypoint to a file
        """
        # Build new waypoint object
        data = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading
        }
        
        # Add to in-memory list
        self.waypoints_list.append(data)

        # Write all waypoints from this session to file
        try:
            with open(self.logging_file_path, 'w') as yaml_file:
                yaml.dump({"waypoints": self.waypoints_list}, yaml_file, 
                         default_flow_style=False)
        except Exception as ex:
            messagebox.showerror(
                "Error", f"Error logging position: {str(ex)}")
            return

        messagebox.showinfo("Info", 
            f"Waypoint logged successfully ({len(self.waypoints_list)} total)")


def main(args=None):
    rclpy.init(args=args)

    # Determine the file path
    if len(sys.argv) > 1:
        # Custom path provided as argument
        yaml_file_path = sys.argv[1]
    else:
        # Default path: faucon_navigation/config/gps_waypoints.yaml
        package_path = get_package_share_directory('faucon_navigation')
        yaml_file_path = os.path.join(package_path, 'config', 'gps_waypoints.yaml')
    
    gps_gui_logger = GpsGuiLogger(yaml_file_path)

    try:
        while rclpy.ok():
            # Spin both the ROS system and the interface
            rclpy.spin_once(gps_gui_logger, timeout_sec=0.1)  # Run ROS2 callbacks
            gps_gui_logger.update()  # Update the tkinter interface
    except KeyboardInterrupt:
        pass  # Arrêt propre avec Ctrl+C
    finally:
        gps_gui_logger.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()