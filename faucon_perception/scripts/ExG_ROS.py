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
        
        # Abonnement à la caméra du robot

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher pour voir le résultat dans ROS
        self.publisher_ = self.create_publisher(Image, '/vision/resultat_rangs', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Nœud de détection lancé (Structure Personnalisée)")

    def image_callback(self, msg):
        try:
            # Conversion ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 1. Appel de TA fonction masking (adaptée pour recevoir l'image)
            img, mask = self.masking(cv_image)
            
            # 2. Appel de TA fonction bird_eye_view
            imgTrans, T = self.bird_eye_view(img, mask)
            
            # 3. Appel de TA fonction ajuster_polynome
        
            result_img, left_fit, right_fit = self.ajuster_polynome(imgTrans)
            
            # Publication du résultat final
            ros_image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            self.publisher_.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Erreur : {e}')

    ## creation du masque avec la méthode d'Otsu 
    def masking(self, img):

        #conversion de l'image
        img_rgb = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

        #separation des canaux de l'image 
        b, g, r = cv2.split(img.astype(np.float32))

        #Calcul de l'indice Excess Green (ExG)
        # Formule : ExG = 2*G - R - B
        exg_img = 2*g -r -b

        #On normalise car les valeurs peuvent etre tres grande voire negative
        #Donc on ramene tout entre 0 et 255
        exg_norm = cv2.normalize(exg_img, None, 0, 255, cv2.NORM_MINMAX)
        exg_norm = exg_norm.astype(np.uint8)

        ret, mask = cv2.threshold(exg_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


        return img, mask

    # computing the perspective view of a slide of the image
    # also known as the bird eyes view 
    def bird_eye_view(self, img, mask):
        # recupere la taille de l'image 
        h, w = img.shape[:2]

        # creation de 4 points sur l'image 
  
        src = np.float32([
            [w * 0.15, h * 0.9],  
            [w * 0.85, h * 0.9],  
            [w * 0.65, h * 0.4],  
            [w * 0.35, h * 0.4]  
        ])
        
        dest = np.float32([
            [w * 0.2, h],       
            [w * 0.8, h],       
            [w * 0.8, 0],       
            [w * 0.2, 0]       
        ])

         # Calcul de la matrice de transformation
        T = cv2.getPerspectiveTransform(src, dest)

        # Application de la transformation sur le masque binaire
        imgTrans = cv2.warpPerspective(mask, T, (w, h), flags=cv2.INTER_LINEAR)


        return imgTrans, T

    # utiliser des fenetres glissantes pour trouver les
    #pixels des rangs gauche et droit 
    def trouver_pixel_rang(self, img_mask):

        # calcul de l'histogramme sur la partie basse de l'image
        histogram = np.sum(img_mask[img_mask.shape[0]//2:, :], axis=0)
        out_img = np.dstack((img_mask, img_mask, img_mask))*255

        #Trouver les pics de l'histogramme gauche et droite 
        midpoint = int(histogram.shape[0]//2)
        leftxbase = np.argmax(histogram[:midpoint])
        rightxbase = np.argmax(histogram[midpoint:]) + midpoint

        #parametres des fenetres glissantes 
        nwindows = 9
        minpix = 40 # nombre de pixel min dans une fenetre
        margin = 60
        window_height = int(img_mask.shape[0]//nwindows)

        #position courrante
        leftx_current = leftxbase
        rightx_current = rightxbase

        # Identifier les pixels non nuls (blancs)
        nonzero = img_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        #liste qui vont stocker les positions des pixels
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            """Définitions des limotes des fenetres en Y et X"""

            #limites de la taille des fenetres en y 
            win_y_low = img_mask.shape[0] - (window+1)*window_height
            win_y_high = img_mask.shape[0] - window*window_height

            # Limites X (Gauche)
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # Limites X (Droite)
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # Dessiner les rectangles (pour visualiser)
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2)         

            # Identifier les pixels dans la fenetre (Dans la liste de 
            # on prend seulement ceux compris entre win_y_low et ..)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0] 

            # Ajouter a la liste 
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # Mise à jour de la position courante 
            # J'ai rajouté ce bloc qui manquait dans ton snippet pour que la fenêtre bouge
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Concaténer les indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        return left_lane_inds, right_lane_inds, out_img, nonzerox, nonzeroy           

    ## Tracer une courbe avec tous les points obtenus grace aux fenetres glissantes 
    def ajuster_polynome(self, img_mask):
        # Trouver les pixels
        left_lane_inds, right_lane_inds, out_img, nonzerox, nonzeroy = self.trouver_pixel_rang(img_mask)

        # Extraire les positions des pixels
        # Sécurité : vérifier qu'on a trouvé des pixels
        if len(left_lane_inds) == 0 or len(right_lane_inds) == 0:
            return out_img, None, None

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]


        # On cherche x = Ay^2 + By + C
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Générer les points x et y pour l'affichage
        ploty = np.linspace(0, img_mask.shape[0]-1, img_mask.shape[0])
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            print('La fonction a échoué à fitter une ligne')
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty

        # Dessiner sur l'image de sortie
        out_img[lefty, leftx] = [255, 0, 0]   # Pixels gauche en Rouge
        out_img[righty, rightx] = [0, 0, 255] # Pixels droite en Bleu
        
        # Dessiner la courbe mathématique (approximation)
        for i in range(len(ploty)-1):
            cv2.line(out_img, (int(left_fitx[i]), int(ploty[i])), (int(left_fitx[i+1]), int(ploty[i+1])), (255, 255, 0), 3)
            cv2.line(out_img, (int(right_fitx[i]), int(ploty[i])), (int(right_fitx[i+1]), int(ploty[i+1])), (255, 255, 0), 3)

        return out_img, left_fit, right_fit 

def main(args=None):
    rclpy.init(args=args)
    node = CornDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()