#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import numpy as np
from rclpy.executors import ExternalShutdownException


# Class used to create a mask 
# from the 2D camera 
# Please mind changing the topics name 
# if using imgaes from the depth camera.
class ImageMasking(Node):
    def __init__(self):
        super().__init__('image_mask')
        #subscribe to /camera/image topic (camera rgbd ; anciennement /camera/image_raw)

        self.subscribe_ = self.create_subscription(Image,'/camera/image', self.image_callback, 10)

        #create a publisher to republish processed images
        self.publisher = self.create_publisher(Image,'/masked_image', 10)
       
        self.cv_bridge = CvBridge()


    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image : {e}")
            return  # sans ce return, cv_image n'existe pas -> NameError
        
        # conversion de RGB en HSV 
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        #limites utilisés pour créer le masque 
        borne_inf = np.array([21, 88, 30])
        borne_sup = np.array([97, 199, 185])

        #création du masque
        mask = cv2.inRange(hsv_img, borne_inf, borne_sup)

        # Appliquer le masque à l'image originale
        result = cv2.bitwise_and(cv_image, cv_image, mask= mask)
        
        #publier l'image résultante 
        processed_msg = self.cv_bridge.cv2_to_imgmsg(result, encoding="bgr8")
        processed_msg.header = msg.header   # horodatage et repere de l'image source
        
        self.publisher.publish(processed_msg)

        

    
def main(args = None):
    rclpy.init(args=args)
    image_converter = ImageMasking()
    try:
        rclpy.spin(image_converter)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    image_converter.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()


