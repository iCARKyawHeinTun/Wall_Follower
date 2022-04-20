#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def get_rotation(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y , orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    print (yaw)

rospy.init_node('quaternion_to_euler')
sub = rospy.Subscriber('/odom', Odometry, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()