#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import numpy as np 
import matplotlib.pyplot as plt
import os 

class CornDetectorNode(Node):
    def __init__(self):
        super().__init__('corn_detector_node')
        
        # Abonnement caméra
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher 1 : Vue Debug (Sliding Windows)
        self.publisher_debug = self.create_publisher(Image, '/vision/debug_sliding_windows', 10)
        
        # Publisher 2 : Résultat Final (Zone verte sur image réelle)
        self.publisher_final = self.create_publisher(Image, '/vision/resultat_final', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Nœud lancé avec visualisation finale")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 1. Masquage
            img, mask = self.masking(cv_image)
            
            # 2. Vue d'oiseau
            # T est la matrice de transformation pour aller vers la vue d'oiseau
            imgTrans, T = self.bird_eye_view(img, mask)
            
            # 3. Polynômes et Sliding Windows
            result_debug, left_fit, right_fit = self.ajuster_polynome(imgTrans)
            
            # 4. Dessiner le résultat final sur l'image originale
            if left_fit is not None and right_fit is not None:
                # On calcule l'inverse de T pour revenir à la vue normale
                Minv = np.linalg.inv(T)
                
                final_image = self.dessiner_resultat_final(cv_image, imgTrans, left_fit, right_fit, Minv)
                
                # Publication du résultat final
                ros_final = self.bridge.cv2_to_imgmsg(final_image, "bgr8")
                self.publisher_final.publish(ros_final)
            
            # Publication du debug 
            ros_debug = self.bridge.cv2_to_imgmsg(result_debug, "bgr8")
            self.publisher_debug.publish(ros_debug)
            
        except Exception as e:
            self.get_logger().error(f'Erreur : {e}')

    ## creation du masque avec la méthode d'Otsu 
    def masking(self, img):
        img_rgb = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        b, g, r = cv2.split(img.astype(np.float32))

        # Excess Green
        exg_img = 2*g -r -b
        exg_norm = cv2.normalize(exg_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        ret, mask = cv2.threshold(exg_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        return img, mask

    # computing the perspective view of a slide of the image
    def bird_eye_view(self, img, mask):
        h, w = img.shape[:2]

        # Points (A ajuster selon ta caméra)
        src = np.float32([
            [w * 0.15, h * 0.9], [w * 0.85, h * 0.9],
            [w * 0.65, h * 0.4], [w * 0.35, h * 0.4]
        ])
        
        dest = np.float32([
            [w * 0.2, h], [w * 0.8, h],
            [w * 0.8, 0], [w * 0.2, 0]
        ])

        T = cv2.getPerspectiveTransform(src, dest)
        imgTrans = cv2.warpPerspective(mask, T, (w, h), flags=cv2.INTER_LINEAR)

        return imgTrans, T

    # utiliser des fenetres glissantes
    def trouver_pixel_rang(self, img_mask):
        histogram = np.sum(img_mask[img_mask.shape[0]//2:, :], axis=0)
        out_img = np.dstack((img_mask, img_mask, img_mask))*255

        midpoint = int(histogram.shape[0]//2)
        leftxbase = np.argmax(histogram[:midpoint])
        rightxbase = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        minpix = 40 
        margin = 60
        window_height = int(img_mask.shape[0]//nwindows)

        leftx_current = leftxbase
        rightx_current = rightxbase

        nonzero = img_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = img_mask.shape[0] - (window+1)*window_height
            win_y_high = img_mask.shape[0] - window*window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2)         

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0] 

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # Mise à jour pour le glissement
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        return left_lane_inds, right_lane_inds, out_img, nonzerox, nonzeroy           

    ## Tracer une courbe
    def ajuster_polynome(self, img_mask):
        left_lane_inds, right_lane_inds, out_img, nonzerox, nonzeroy = self.trouver_pixel_rang(img_mask)

        if len(left_lane_inds) == 0 or len(right_lane_inds) == 0:
            return out_img, None, None

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, img_mask.shape[0]-1, img_mask.shape[0])
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty

        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]
        
        for i in range(len(ploty)-1):
            cv2.line(out_img, (int(left_fitx[i]), int(ploty[i])), (int(left_fitx[i+1]), int(ploty[i+1])), (255, 255, 0), 3)
            cv2.line(out_img, (int(right_fitx[i]), int(ploty[i])), (int(right_fitx[i+1]), int(ploty[i+1])), (255, 255, 0), 3)

        return out_img, left_fit, right_fit 

  
    # Permet de redessiner la zone détectée sur l'image d'origine
    def dessiner_resultat_final(self, original_img, warped_img, left_fit, right_fit, Minv):
        # Créer une image vide pour dessiner la forme
        warp_zero = np.zeros_like(warped_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Générer les points Y
        ploty = np.linspace(0, warped_img.shape[0]-1, warped_img.shape[0])
        
        # Calculer les points X des polynômes
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        # Formater les points pour OpenCV fillPoly
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Dessiner le polygone vert sur l'image vue d'oiseau
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Inverser la perspective (Retour à la vue normale)
        newwarp = cv2.warpPerspective(color_warp, Minv, (original_img.shape[1], original_img.shape[0])) 
        
        # Superposer sur l'image originale
        result = cv2.addWeighted(original_img, 1, newwarp, 0.3, 0)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CornDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()