#! /usr/bin/env python
import rospy
import actionlib
from wall_following.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
# from std_msgs.msg import Empty

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# def feedback_callback(feedback):
#     my_feedback = OdomRecordFeedback()
#     rospy.loginfo('inside feedback loop...')
#     print(my_feedback)

def feedback_callback(feedback):
    state_result = client.get_state()
    rospy.loginfo('still in progress...')
    while state_result < DONE :
        state_result = client.get_state()
        rate.sleep()
        my_feedback = OdomRecordFeedback()
        rospy.loginfo(my_feedback)

rospy.init_node('Record_Odom_Client')
client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
client.wait_for_server()
rate = rospy.Rate(1)

client.send_goal({},feedback_cb = feedback_callback)
client.wait_for_result()

state_result = client.get_state()
rospy.logwarn(state_result)


rospy.loginfo('[Result] State:' +str(state_result))
if state_result == ERROR:
    rospy.logerr('Something went wrong in the Server Side')
if state_result == WARN : 
    rospy.logwarn('There is a warning in the Server Side')