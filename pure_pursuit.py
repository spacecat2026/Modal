#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# TODO: import ROS msg types and libraries

import tf
from math import*
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """

    def __init__(self):
        #le coeff de proportionnalité entre le steering_angle et la courbature
        self.k = 0.6
        self.L = 0.4
        self.velocity = 0.8
        self.data=[]

        #fichier csv avec les waypoints
        self.waypoints = np.loadtxt('/home/eva/rcws/logs/wp-2026-02-18-14-41-16.csv', delimiter=',', usecols=(0, 1))
        self.waypoints_x = self.waypoints[:, 0]  # Toutes les lignes, colonne 0
        self.waypoints_y = self.waypoints[:, 1]  # Toutes les lignes, colonne 1


        self.sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/visual_goal', Marker, queue_size=1)

    def pose_callback(self, pose_msg):
        #coordonnées de la voiture
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        # TODO: find the current waypoint to track using methods mentioned in lecture
        #on crée la liste des distances au véhicules, en supposant que self.data est une liste de waypoint (x,y) ?? en fait ce sera un fichier csv a traiter avec pandas, mais on voit ca après
        dists = np.sqrt((self.waypoints_x - x)**2 + (self.waypoints_y - y)**2)

        # On cherche les points devant nous (distance >= L)
        indices = np.where(dists >= self.L)[0]

        if len(indices) == 0: # Si on est à la fin du fichier
            return

        min_idx = indices[0]
        x0 = self.waypoints_x[min_idx]
        y0 = self.waypoints_y[min_idx]

        # TODO: transform goal point to vehicle frame of reference
        #on cherche y
        #acces à l'angle de la voiture avec tf et le quaternion
        quaternion = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # TODO: calculate curvature/steering angle
        #angle = euler[2]
        #distance entre la voiture et le waypoint goal
        l = sqrt((x-x0)**2 + (y-y0)**2)
        #angle entre le way point et l'axe X pointant vers l'est du repère.
        alpha = atan2(y0-y,x0-x)
        delta = alpha - euler[2]
        y = sin(delta)*l
        gamma = 2*y/l**2
        angle = self.k*gamma
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        if abs(angle) > 0.4189:
            angle = np.sign(gamma)*0.4189

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.velocity if abs(angle)<0.2 else 0.5*self.velocity
        self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()