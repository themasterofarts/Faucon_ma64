#!/usr/bin/env python3

import math
import numpy as np

import time

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 


from actuator_msgs.msg import Actuators  # topic pour publier au controller des moteurs des drone



class PidControllerDrone(Node):
    def __init__(self):
        super().__init__("Pid_Controller_Drone")
        
        
        self.get_logger().info("Pid_Controller_Drone stardet")
        
        ##### abonner aux different topic TWist et odom ou Imu

            ## topic Twist
        self.sub_cmd = self.create_subscription(Twist,
                                                 "cmd_vel",
                                                self.cmd_vel_callback,10) ### abonner topic cmd_vel pour recupére la vitesse lineaire et angulaire
        self.sub_cmd
                
            ## Topic Imu  et odm #####
        self.sub_imu = self.create_subscription(Imu,
                                                "/drone/imu",
                                                self.imu_callback,
                                                10)  ### abonner topic cmd_vel pour savoir la pose du drone en permanence
        self.sub_imu
    

        self.sub_odom = self.create_subscription(Odometry,
                                                "/faucon_drone/odom",
                                                self.odom_callback,
                                                10)  ### abonner topic cmd_vel pour savoir la pose du drone en permanence
        self.sub_odom 
        
        ##### tpoic pour publier au motor du drone #######
        
        self.pub= self.create_publisher(Actuators,
                                        "/faucon_drone/command/motor_speed",
                                        10,)
        
        
        ## timer callback
        
        #timer_period = 0.01 #seconde
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        #Initialisation du temps pour le cacul du dt
        self.last_time = time.time()
        
        ### initialisatio des parametres
        
        self.v = 0.0 ## vitesse lineaire en m/s
        self.w = 0.0 ## vitesse angulaire en rad/s
        
        
        #### initialisation des angles de euleurs #####
        self.altitude_Z =0.0      ### savoir l'altitude du drone
        
        self.roll = 0.0 ### anles de rouli
        self.pitch = 0.0  ###  angles de pitch 
        self.yaw = 0.0 #### angless de yaw
        
        
        # Paramètres physiques  du drone
        self.masse_drone = 1.2  # kg
        self.gravite = 9.81     # m/s^2
        self.portance = 8.54858e-06 # Le coefficient de portance defini dans le plugin des moteurs
        self.l_dist_motor = 0.17 ### la distance entre moteur en metre(pythagore)
        self.d = 0.016  ## coef de trainé du yaw  
        
        
        ############################################
        #      Paramètres PID Altitude z           #
        ############################################
        self.cible_z = 5  # consigne pour atteidre une altitude z=5m
        self.kp_z = 2.0    # pour pousser le drone vers la cible 
        self.kd_z = 0.5    # Le frein # empècher le dépassement
        self.ki_z = 0.0  ## pour corriger l'erreur
        self.integral_z =0.0 ##
        
        self.erreur_z_precedente = 0.0
        self.hover_thrust = self.masse_drone * self.gravite # Force de base pour contrer la gravité (dépend du poids de ton drone)
        
        ############################################
        #      Paramètres PID pour le roll          #
        ############################################
        
        self.roll_cible = 0.0 # en radiant
        self.kp_roll = 1.0 ##  pour pousser le drone vers la cible
        self.kd_roll = 0.01 ##  pour s'assurer que le motor deu drne ne depasse pas roll_cible
        self.ki_roll = 1.0 ## pour assure la precision de erreur
        self.erreur_roll_preced = 0.0
        self.integral_roll =0.0 
        
        ############################################
        #      Paramètres PID pour le pitch          #
        ############################################
        
        self.pitch_cible = 0.0 # en radiant
        self.kp_pitch = 1.0 ##  pour pousser le drone vers la cible
        self.kd_pitch = 0.01 ##  pour s'assurer que le motor deu drne ne depasse pas roll_cible
        self.ki_pitch = 1.0 ## pour assure la precision de erreur
        self.erreur_pitch_preced = 0.0
        self.integral_pitch =0.0 
        
        ############################################
        #      Paramètres PID pour le yaw          #
        ############################################
        
        self.yaw_cible = 0.0 # en radiant
        self.kp_yaw = 0.5 ##  pour pousser le drone vers la cible
        self.kd_yaw = 0.01 ##  pour s'assurer que le motor deu drne ne depasse pas roll_cible
        self.ki_yaw = 0.0 ## pour assure la precision de erreur
        self.erreur_yaw_preced = 0.0
        self.integral_yaw = 0.0 
        
        

        
        
    def conv_euler_from_quaternion(self, x, y, z, w):
        """Convertit un quaternion en angles d'Euler (roll, pitch, yaw)"""
        
        
        ## calcule de l'angle roll
        sin_roll = +2.0 * (w * x + y * z) ### calcule de la projection  mathématique pour avoir l'angle sinus
        cos_roll = +1.0 - 2.0 * (x * x + y * y)  ### calcule de la projection  mathématique pour avoir l'angle communicataion
        roll_x = np.arctan2(sin_roll, cos_roll)  ## calcule de roll inclinaisont(gauche /droite) selon l'axe x
     
        ##### calcule de l'angle picth #####
        sin_pitch = 2 * (w * y - z * x)  ### calcule de la projection  mathématique 
                                    ###   pour avoir l'angle sinus et isoler la rotation autor de y
        
        if sin_pitch > 1.0:    ### on fait cette condition pour eviter un crash 
                               ### car arcsin ne renvoit les valeur compris en -1 et 1
            sin_pitch = 1.0
            
        elif sin_pitch<-1.0:
            sin_pitch = -1.0
        pitch_y = np.arcsin(sin_pitch)   ###    calcule de roll inclinaisont(avant/arrière) selon l'axe y
     
     
        ##### calcule de l'angle yaw #####
        sin_yaw = +2.0 * (w * z + x * y)
        cos_yaw = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(sin_yaw, cos_yaw)
     
        return roll_x, pitch_y, yaw_z # avoir les angles d'euleur en ra dians 
    
       
   #### Methode pour ecouter les vitese line er angulaire##############    
    def cmd_vel_callback(self, msg:Twist):
        
        self.v =  msg.linear.x   ### vitesse lineaire
        self.w = msg.angular.z ### vitesse angulair
        
        
    def imu_callback(self,msg:Imu):
        pass
        
        
####Methode pour ecouter les angles d'euler######
    def odom_callback(self, msg:Odometry):
        
        self.altitude_Z = msg.pose.pose.position.z  ## recupéré l'altitude du drone
        
        q= msg.pose.pose.orientation
        
        # récupuré les valeurs des angles d'euleur
        self.roll,self.pitch,self.yaw = self.conv_euler_from_quaternion(q.x, q.y, q.z,q.w)
        
        ### convertir c'est angles en dégrée######
        
        self.roll_deg = (self.roll*180)/np.pi ## convertir l'angle d'eulur en dégré
        self.pitch_deg = (self.pitch*180)/np.pi ## convertir l'angle d'eulur en dégré
        self.yaw_deg= (self.yaw*180)/np.pi ## convertir l'angle d'eulur en dégré
        
        self.get_logger().info(f"Roll: {self.roll_deg:.2f}° | Pitch: {self.pitch_deg:.2f}° | Yaw: {self.yaw_deg:.2f}°")
        
    ################################### 
    #  Coeur ou le PID est implémenter #
    ###################################
    
    #def timer_callback(self):
        
        
        #current_time = time.time()
        #dt = current_time - self.last_time
        dt = 0.01 ## ficer la variation dt
        
        if dt <= 0:
            return
        
        
        # Pencher en avant pour avancer (pitch négatif)
        self.pitch_cible = -np.clip(self.v * 0.15, -0.3, 0.3)  # ±17°
        
        # Intégrer vitesse angulaire pour yaw
        self.yaw_cible += self.w * dt
        
        self.yaw_cible = np.arctan2(np.sin(self.yaw_cible), 
                                     np.cos(self.yaw_cible))
        
        ##############################
        # PID pour l'altitude Z       #
        ###############################
        
        ### erreur ####
        erreur_z = self.cible_z - self.altitude_Z
        
        ### proportiel ####
        p_z = self.kp_z * erreur_z  
        
        ### dériver ###
        
        d_z = self.kd_z* ( (erreur_z - self.erreur_z_precedente)/dt )
        
        #### l'intégrale ####
        self.integral_z +=erreur_z*dt 
        self.integral_z = np.clip(self.integral_z, -10, 10)  # Anti-windup
        
        
        i_z = self.ki_z * self.integral_z 
        
        
        ### force de pousser pour tiréb le drone vers le haut ###
        u_1 = i_z + d_z + p_z + self.hover_thrust
        
        ### mise  à jour##### 
        self.erreur_z_precedente = erreur_z  ## mise a jour de l'erreur
         
        #self.last_time = current_time
        
        
        ##################################
        # PID pour l'angle d'euleur roll #
        ##################################
        
        e_roll = self.roll_cible - self.roll
        
        ## proportionel ####
        p_roll = self.kp_roll * e_roll
        
        ### derive roll ####
        
        d_roll = self.kd_roll * ( (e_roll - self.erreur_roll_preced) /dt )
        
        #### intégral ############
        
        self.integral_roll += e_roll * dt ## somme des erreurs
        self.integral_roll = np.clip(self.integral_roll, -1.0, 1.0)
        
        i_roll = self.ki_roll *self.integral_roll
        
        #### la force pour pencher le drone vers le drone doite/gauche
        
        u_2  = i_roll + d_roll + p_roll  ## force total
        
         ### mis ajour #############
        self.erreur_roll_preced = e_roll
        
        ##################################
        # PID pour l'angle d'euleur pitch #
        ##################################
        
        e_pitch = self.pitch_cible - self.pitch
        
        ## proportionel pitch ####
        p_pitch = self.kp_pitch * e_pitch
        
        ### derive pitch ####
        
        d_pitch = self.kd_pitch * ( (e_pitch - self.erreur_pitch_preced) /dt )
        
        #### intégral ############
        
        self.integral_pitch += e_pitch * dt ## somme des erreurs
        self.integral_pitch = np.clip(self.integral_pitch, -1.0, 1.0)
        
        i_pitch = self.ki_pitch * self.integral_pitch 
        
        #### la force pour pencher le drone vers le drone doite/gauche
        
        u_3  = i_pitch + d_pitch + p_pitch  ## force total  pour les 4 moteurs
        
        ### mis ajour #############
        self.erreur_pitch_preced = e_pitch
        
        
        
        
        ##################################
        # PID pour l'angle d'euleur yaw #
        ##################################
        
        e_yaw = self.yaw_cible - self.yaw
        
        ## proportionel yaw ####
        p_yaw = self.kp_yaw * e_yaw
        
        ### derive yaw ####
        
        d_yaw = self.kd_yaw * ( (e_yaw - self.erreur_yaw_preced) /dt )
        
        #### intégral ############
        
        self.integral_yaw += e_yaw * dt ## somme des erreurs
        self.integral_yaw = np.clip(self.integral_yaw, -5, 5)
        
        i_yaw = self.ki_yaw * self.integral_yaw
        
        #### la force pour pencher le drone vers le drone doite/gauche
        
        u_4  = i_yaw + d_yaw + p_yaw  ## force total  pour les 4 moteurs
        
        #### mise à jour###
        self.erreur_yaw_preced = e_yaw
        
        
        ##########################""
        # matrice de mixage #
        ###########################
        # on sum u_1,u_2,u_3 et u_4 en foction de leus poses 
        # Force = (Poussée) + (Roulis) + (Tangage) + (Lacet)
        
        force_m1 = (u_1/4) + (u_2/(4)) - (u_3/(4)) + (u_4/4) # Avant-Gauche (FL)
        force_m2 = (u_1/4) - (u_2/(4)) - (u_3/(4)) - (u_4/4) # Avant-Droit (FR)
        force_m3 = (u_1/4) + (u_2/(4)) + (u_3/(4)) - (u_4/4) # Arrière-Gauche (BL)
        force_m4 = (u_1/4) - (u_2/(4)) + (u_3/(4)) + (u_4/4) # Arrière-Droit (BR)
        


        ################################
        # vitesse de rotation (rad/s)  #
        # ##############################
        
        # Si le PID demande une force négative, on coupe le moteur (0.0) au lieu de l'inverser !
        force_m1 = max(0.0, force_m1)
        force_m2 = max(0.0, force_m2)
        force_m3 = max(0.0, force_m3)
        force_m4 = max(0.0, force_m4)
        
        w1 = math.sqrt(force_m1 / self.portance)
        w2 = math.sqrt(force_m2 / self.portance)
        w3 = math.sqrt(force_m3 / self.portance)
        w4 = math.sqrt(force_m4 / self.portance)
        
                # Saturation selon ton plugin XML
        MAX_MOTOR_SPEED = 1000.0  # rad/s
        w1 = np.clip(w1, 0, MAX_MOTOR_SPEED)
        w2 = np.clip(w2, 0, MAX_MOTOR_SPEED)
        w3 = np.clip(w3, 0, MAX_MOTOR_SPEED)
        w4 = np.clip(w4, 0, MAX_MOTOR_SPEED)
        
        ################################
        #      Publication   #
        # ##############################
        msg_moteurs = Actuators()
        msg_moteurs.velocity = [w1, w2, w3, w4]
        self.pub.publish(msg_moteurs)
        
        
               
        
        # === FONCTION PRINCIPALE POUR DÉMARRER LE NOEUD ===
def main(args=None):
    rclpy.init(args=args)
    node = PidControllerDrone()
    
    try:
        rclpy.spin(node) # Fait tourner la boucle à l'infini
    except KeyboardInterrupt:
        pass # Permet de quitter proprement avec Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()