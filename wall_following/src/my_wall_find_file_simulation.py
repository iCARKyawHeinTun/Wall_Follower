#! /usr/bin/env python3

import time
import rospy
from wall_following.srv import FindWall, FindWallResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def my_service_callback(request):
    rospy.loginfo('searching the wall...')
    flag = 0
    while flag == 0:
        pub.publish(obj)
        if obj.angular.z == 0:
            flag = 1
    if flag == 1:
        pub.publish(obj)
    rospy.loginfo('facing the wall...')
    obj.angular.z = 0.5
    pub.publish(obj)
    time.sleep(2.8)
    obj.angular.z = 0
    pub.publish(obj)
    rospy.loginfo('ready for wall following...')
    my_find_wall_response = FindWallResponse()
    my_find_wall_response.wallfound = True
    return my_find_wall_response

def my_sub_callback(msg):
    rospy.get_master
    if min(msg.ranges) != min(msg.ranges[0:5]): 
        obj.angular.z = 0.2
    elif min(msg.ranges) == min(msg.ranges[0:5]): 
        obj.angular.z = 0

rospy.init_node('my_wall_find_service')
my_service = rospy.Service('/find_wall',FindWall, my_service_callback)
sub = rospy.Subscriber('/scan', LaserScan, my_sub_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
obj  = Twist()
rate = rospy.Rate(10)
rospy.spin()

# rosrun wall_following my_wall_find_file_simulation.py

# rosservice call /find_wall "{}" 