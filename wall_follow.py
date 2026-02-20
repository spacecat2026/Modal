#!/usr/bin/env python3
import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow:
    def __init__(self):
        rospy.init_node("WallFollow_node", anonymous=True)

        # Topics
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Paramètres PID
        self.kp = 1.0
        self.kd = 0.05
        self.ki = 0.0

        # Paramètres de navigation
        self.DESIRED_DISTANCE = 0.8  # Distance au mur gauche (m)
        self.VELOCITY = 2.0
        self.LOOKAHEAD_DIST = 1.0    # Projection dans le futur

        self.prev_error = 0.0
        self.integral = 0.0

        # Angles fixes pour le mur GAUCHE (0=Face, 90=Gauche)
        self.angle_b = 90.0
        self.angle_a = 45.0

    def getRange(self, data, angle_deg):
        """ Récupère la distance pour un angle donné en degrés """
        target_rad = math.radians(angle_deg)
        index = int((target_rad - data.angle_min) / data.angle_increment)

        if index < 0 or index >= len(data.ranges):
            return 10.0

        dist = data.ranges[index]
        if math.isinf(dist) or math.isnan(dist) or dist <= 0:
            return 10.0
        return dist

    def lidar_callback(self, data):
        """ Calcul de l'erreur et exécution du PID """
        # 1. Récupération des deux rayons pour le mur gauche
        a = self.getRange(data, self.angle_a)
        b = self.getRange(data, self.angle_b)
        theta = math.radians(self.angle_b - self.angle_a)

        # 2. Calcul de alpha (orientation de la voiture par rapport au mur)
        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))

        # 3. Calcul de la distance actuelle et future
        dist_current = b * math.cos(alpha)
        dist_predicted = dist_current + self.LOOKAHEAD_DIST * math.sin(alpha)

        # 4. Calcul de l'erreur (Consigne - Mesure)
        error = -(self.DESIRED_DISTANCE - dist_predicted)

        # 5. Appel du contrôleur
        self.pid_control(error)

    def pid_control(self, error):
        """ Génère la commande de direction """
        # Calcul des composantes PID
        P = self.kp * error
        self.integral += error
        D = self.kd * (error - self.prev_error)
        self.prev_error = error

        # Angle de braquage (inversé pour ROS : positif = gauche, négatif = droite)
        # Si error > 0 (trop loin du mur gauche), on doit braquer à gauche (+)
        steering_angle = (P + D + (self.ki * self.integral))

        # Limitation physique de l'angle (~20 degrés)
        steering_angle = max(min(steering_angle, 0.34), -0.34)

        # Publication du message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.VELOCITY
        self.drive_pub.publish(drive_msg)

def main(args):
    wf = WallFollow()
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)