#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class GnssWaypointCollector(Node):
    def __init__(self):
        super().__init__("gnss_waypoint_collector")
        
        self.declare_parameter("waypoints_file", "")
        waypoints_file = self.get_parameter(
            "waypoints_file").get_parameter_value().string_value
        
        # If no file specified, use default from package
        if not waypoints_file:
            package_share = get_package_share_directory('faucon_navigation')
            waypoints_file = os.path.join(package_share, 'config', 'waypoints_gnss.yaml')
            self.get_logger().info(f"Using default waypoints file: {waypoints_file}")
        
        if not os.path.exists(waypoints_file):
            self.get_logger().error(
                f"GNSS waypoints file not found: '{waypoints_file}'")
            raise RuntimeError("GNSS waypoints file not found.")
        
        self.waypoints = self.load_waypoints(waypoints_file)
        
        self.get_logger().info(
            f"{len(self.waypoints)} GNSS waypoints loaded.")
        
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"[{i}] lat={wp['lat']} lon={wp['lon']}")
    
    def load_waypoints(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        
        if "waypoints" not in data:
            raise RuntimeError("Missing 'waypoints' key in YAML file")
        
        return data["waypoints"]

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GnssWaypointCollector()
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

if __name__ == "__main__":
    main()