#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def my_callback(msg):
    distance_moved = msg.data
    if msg.data  < 2 : 
        move.linear.x = 0.1
    if msg.data > 2 : 
        move.linear.x = 0

    pub.publish(move)

rospy.init_node('test_movement')
sub = rospy.Subscriber('/moved_distance',Float64,my_callback)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
move = Twist()
rospy.spin()