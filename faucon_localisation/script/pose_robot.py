import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion



class PoseRobot(Node):
    
    
    def __init__(self):
        super().__init__("position_robot")
        
        self.get_logger().info("Node position robot started")
        
        
        #### s'aboonner pour récupéré la pose du robot
        
        self.sub = self.create_subscription(Odometry, "odometry/filtered", self.pose_robot_callback, 10)
        
        
        
        
        def pose_robot_callback(self, msg) :
            
            ### recuperation de la pose local du robot 
            
            x_robot = msg.pose.pose.position.x
            y_robot = msg.pose.pose.position.y
            
            
            ### rotation selon x, y, z
            quat = msg.pose.pose.orientation
            yaw_local = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            self.get_logger().info( f"  position local  [x_robot= {x_robot:.3f}m ,  y_robot= {y_robot:.3f}m ],yaw_local= {yaw_local} ")
            
            
def main():
    rclpy.init()
    node =PoseRobot()
    rclpy.spin(node)