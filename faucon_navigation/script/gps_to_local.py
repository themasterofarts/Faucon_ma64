import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import utm
import math



class GnssToLocal(Node):
    def __init__(self):
        super().__init__("gnss_to_local")
        self.get_logger().info("Node convertion gnss to local started")
        
        #### initialisation des coordonné local ###
        
        
        self.origin_set = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw = 0.0
        
        
        #### abonner au topic gnss et Imu ####
        
        self.subscriber_1 = self.create_subscription(NavSatFix,"gnss/fix", self.gnss_to_local_callback,10)
        
        self.subcriber_2 = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        
        ##### publier au topic /odom #####
        
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        
    
    def gnss_to_local_callback(self, msg):
        
        lat = msg.latitude
        long = msg.longitude
        
        
        #### conversion de gnss en global(utm) en metre ####
        
        easting, northing, zone, letter = utm.from_latlon(lat, long)
        
        
        #### definition de l'origine local ####
        
        if not self.origin_set:
            
            self.x0 = easting
            self.y0 = northing 
            self.origin_set= True
            
            self.get_logger().info(f"origine local: zone={zone}, letter={letter}")
            
            #return
            
        
        #### convertion de utm en local ####
        
        x_local = easting - self.x0
        y_local = northing - self.y0
        
        self.get_logger().info(f"position local: x={x_local:.2f} m, y={y_local:.2f} m" )
        
        
        ###### publier les coordonné local sur l'odom  ######
        
        odome = Odometry()
        
        odome.header.stamp = self.get_clock().now().to_msg()
        odome.header.frame_id = "map"
        odome.child_frame_id = "base_link"
        
        odome.pose.pose.position.x = x_local
        odome.pose.pose.position.y = y_local
        odome.pose.pose.position.z = 0.0
        
        ##### Orientation depuis IMU #####
        
        q = self.yaw_to_quaternion(self.yaw)
        odome.pose.pose.orientation = q
        
        #### publication sur l'odometry ####
        
        self.odom_pub.publish(odome)
        
        
        
        ##### Methode  pour l'imu #####
        
    def imu_callback(self,msg):
            
            q = msg.orientation
            
            self.yaw = self.quaternion_to_yaw(q)
            
        ### fonction pour la convertion de quaternion à yaw : c'est convertir la rotation defini sur ros en rotation autour de l'axe z (angle d'euler vue que rons ne connais pas ces angles)
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            
        return math.atan2(siny_cosp, cosy_cosp)   
        
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q  
        
        
def main():
    rclpy.init()
    node = GnssToLocal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
        
        
        
        
        
        
        
        
        
        
        
            
            
            
            
        
        



