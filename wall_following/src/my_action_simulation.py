#!/usr/bin/env python3

import math
import rospy
import actionlib
from wall_following.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


class RecordedOdomClass(object):
    result = Point()
    NewPosition = Point()
    _result_record = OdomRecordResult()
    _feedback = OdomRecordFeedback()
    _feedback.current_total = 0.0
    theta = 0.0

    def __init__(self):
        self._result_record = OdomRecordResult()
        rospy.loginfo('Odom Record Action Server is ready...')
        self._record_server = actionlib.SimpleActionServer("record_odom", OdomRecordAction, self.record_callback, False)
        self._record_server.start()
        rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.get_init_position()
        
    def get_init_position(self):
        data_odom = None
        try:
            data_odom = rospy.wait_for_message('/odom',Odometry,timeout=1)
        except:
            rospy.loginfo('Current odom not ready yet, retrying for setting up init pose')
        self.result.x = data_odom.pose.pose.position.x
        self.result.y = data_odom.pose.pose.position.y

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x , orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        self.NewPosition = msg.pose.pose.position
        self._feedback.current_total += self.calculate_distance(self.NewPosition, self.result)
        self.updatecurrent_position(self.NewPosition)
        odom_readings = Point()
        odom_readings.x = self.NewPosition.x
        odom_readings.y = self.NewPosition.y
        odom_readings.z = theta
        self._result_record.list_of_odoms.append(odom_readings)

    def updatecurrent_position(self, new_position):
        self.result.x = new_position.x
        self.result.y = new_position.y

    def calculate_distance(self, new_position, old_position):
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

    def record_callback(self,goal):
        r = rospy.Rate(1)
        success = True

        while self._feedback.current_total < 15:
            self._record_server.publish_feedback(self._feedback)
            rospy.loginfo('completed distance = %s m',self._feedback.current_total)
            r.sleep()

        rospy.loginfo('Completed one lap!!!')

        if success:
            self._record_server.set_succeeded(self._result_record)

if __name__ == '__main__':
    rospy.init_node('my_action_node')
    RecordedOdomClass()
