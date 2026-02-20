#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.lidar_topic = '/scan'
        self.drive_topic = '/nav'
        #self.lidar_topic ='scan'
        #self.drive_topic = '/vesc/ackermann_cmd_mux/input/-navigation'
        self.k=0.6 #coefficient de gain sur le steering_angle pour éviter qu'il ne soit trop grand (zigzag de la voiture)

        self.gap_threshold=0.4
        self.r=0.3
        self.max_speed = 3.0
        self.w=5
        self.angle = 70
        self.max_lookahead=5.0
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)
        self.steering_prec =0.0

    def preprocess_lidar(self, ranges, data):
        proc_ranges=np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)]=data.range_max
        proc_ranges[np.isnan(proc_ranges)]=0.0
        #on regarde pas plus loin que self.max_lookahead
        proc_ranges=np.clip(proc_ranges, 0, self.max_lookahead)
        return proc_ranges

    def find_max_gap(self, free_space_ranges,gap_threshold):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # n=len(free_space_ranges)
        # start_i=0
        # end_i=0
        # curr_start_i=0
        # for i in range(n):
        #     if free_space_ranges[i]==0:
        #         if i - curr_start_i > end_i - start_i:
        #             start_i = curr_start_i
        #             end_i = i
        #         curr_start_i=i+1
        # if n - curr_start_i > end_i - start_i:
        #     start_i = curr_start_i
        #     end_i = n
        # return (start_i, end_i)
        #autre version plus longue à écrire mais moins en memoire complexité
        #j'ai oublié
        M1 = free_space_ranges > gap_threshold

        if not np.any(M1):
            return None, None

        M2 = np.concatenate(([False], M1, [False]))
        M3 = np.diff(M2.astype(int))
        debuts= np.where(M3==1)[0]
        fins = np.where(M3==-1)[0]

        longueurs = fins - debuts
        max_gap_ind = np.argmax(longueurs)

        return debuts[max_gap_ind],fins[max_gap_ind]


    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        if end_i - start_i == 0:
            return start_i
        gap_ranges = ranges[start_i:end_i]

        max_i=np.argmax(np.array(ranges[start_i:end_i]))
        return int(start_i + 0.5*(0.5*(start_i+end_i)))
        #return int(start_i + 0.5*max_i + 0.5*(0.5*(start_i+end_i)))

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges,data)



        #only look for angles between +- self.angle
        n=int(math.radians(self.angle)/data.angle_increment)
        view_ranges = proc_ranges[n:-n]
        angle_ind_zero = data.angle_min + (n*data.angle_increment)
        #Find closest point to LiDAR
        closest_ind=np.argmin(np.array(view_ranges))

        #Eliminate all points inside 'bubble' (set them to zero)
        d=view_ranges[closest_ind]#distance minimale
        beta = math.atan2(self.r, d) if d> 0.1 else math.pi/2
        m=int(beta/data.angle_increment)
        view_ranges[max(0,closest_ind-m):min(closest_ind + m,len(view_ranges))]=0.0
        #Find max length gap
        start_gap, end_gap = self.find_max_gap(view_ranges, self.gap_threshold)
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"

        if start_gap is None:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        #Find the best point in the gap
        best_point=self.find_best_point(start_gap, end_gap, view_ranges)

        #target_point = best_point + deb_i
        #steering_angle = data.angle_min + target_point*data.angle_increment
        steering_angle = angle_ind_zero+ (best_point)*data.angle_increment

        #steering_angle = max(min(steering_angle, 0.35), -0.35)
        # print(steering_angle)
        # smoothing = 0.7
        # steering_angle = (smoothing * steering_angle) + ((1.0 - smoothing) * self.steering_prec)
        # self.steering_prec = steering_angle


        drive_msg.drive.steering_angle =   steering_angle*self.k
        print(steering_angle)
         # Calculate Speed
        speed = self.max_speed if abs(steering_angle) <0.2 else 0.5*self.max_speed
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)