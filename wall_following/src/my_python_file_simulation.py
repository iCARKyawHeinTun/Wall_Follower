#! /usr/bin/env python3

import time
import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_following.srv import FindWall, FindWallRequest
from wall_following.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from std_srvs.srv import Empty

obj = Twist()
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

def action_feedback_callback(feedback):
    state_result = action_client.get_state()

    while state_result < DONE :
        state_result = action_client.get_state()

    result_record = action_client.get_result()
    rospy.loginfo('positions and orientations are : %s ',result_record)
    rospy.loginfo('wall following still in progress...')

def callback(msg):

    obj.linear.x = 0.15

    if msg.ranges[270] > 0.4:
        obj.angular.z = -0.1
    if msg.ranges[270] < 0.2:
        obj.angular.z = 0.1
    if msg.ranges[270] < 0.4 and msg.ranges[270] > 0.2:
        obj.angular.z = 0
    if msg.ranges[0] < 0.7:
        obj.angular.z = 1


rospy.init_node('my_node_wall')

rate = rospy.Rate(1)

rospy.wait_for_service('/find_wall')
find_wall_client = rospy.ServiceProxy('/find_wall',FindWall)
action_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
action_client.wait_for_server()
time.sleep(3)

rospy.loginfo('service in prograss...')
result = find_wall_client()
rospy.loginfo('Service Finished!!!')
rospy.loginfo('wall following in progress...')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

action_client.send_goal({},feedback_cb = action_feedback_callback)

while not rospy.is_shutdown():
    pub.publish(obj)
    #rate.sleep()

# rosrun wall_following my_python_file_simulation.py