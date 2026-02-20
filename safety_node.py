#!/usr/bin/env python
import rospy
import math
#import ROS msg types and libraries
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.ttc_threshold = 0.8
        #create ROS subscribers and publishers.
        #Pour la f1tenth
        #self.brake_pub= rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        #self.brake_bool_pub= rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', Bool, queue_size=10)

        #Pour Rviz
        self.brake_pub= rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub= rospy.Publisher('/brake_bool', Bool, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)



    def odom_callback(self, odom_msg):
        #update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        Compute TTC and brake if necessary
        """
        if self.speed <= 0:
            return
        min_ttc = float('inf')

        for i, r in enumerate(scan_msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            projected_speed = self.speed * math.cos(angle)

            if projected_speed <= 0:
                continue

            ttc = r / projected_speed
            min_ttc = min(min_ttc, ttc)

        # Emergency brake
        if min_ttc < self.ttc_threshold:
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0

            self.brake_pub.publish(brake_msg)
            self.brake_bool_pub.publish(Bool(data=True))
            print("BRA")
        else:
            self.brake_bool_pub.publish(Bool(data=False))

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()