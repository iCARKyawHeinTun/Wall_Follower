#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class MovementDetector(object):
    def __init__(self):
        self._moved_distance = Float64()
        self._moved_distance.data = 0.0
        
        self.get_init_position()
        self.distance_moved_pub = rospy.Publisher('/moved_distance',Float64,queue_size=1)
        rospy.Subscriber('/odom',Odometry, self.odom_callback)

    def get_init_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message('/odom',Odometry,timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
                
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self, msg):
        NewPosition = msg.pose.pose.position
        self._moved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        self.updatecurrent_position(NewPosition)

        if self._moved_distance.data < 0.000001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._moved_distance)
    
    def updatecurrent_position(self,new_position):
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2-x1,y2-y1)
        return dist
    
    def publish_moved_distance(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('movement_detector_node', anonymous= True)
    movement_obj = MovementDetector()
    movement_obj.publish_moved_distance()
