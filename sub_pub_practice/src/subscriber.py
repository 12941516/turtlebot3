#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String, Int32
import os
import sys

class SubNode:
    def __init__(self):
        rospy.init_node('sub_node')
        self.r = rospy.Rate(1)
        self.is_end = False
        
        self.pub = rospy.Publisher('guess', Int32, queue_size = 1)
        rospy.Subscriber('test', String, self.callback, queue_size=1)
        rospy.Subscriber('end', Int32, self.is_end_callback, queue_size=1)
        
    def callback(self, data):
        rospy.loginfo(data.data)
        
    def is_end_callback(self, data):
        if(data.data == -1):
            self.is_end = True
    def run(self):
        while not rospy.is_shutdown() and not self.is_end:
            number = int(input())
            self.pub.publish(number)

if __name__ == '__main__':
    node = SubNode()
    node.run()
