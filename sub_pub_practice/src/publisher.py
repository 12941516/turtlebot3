#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String, Int32

class NumberPublisher:
    def __init__(self):
        rospy.init_node('pub_node')
        self.number = random.randint(1,100)
        self.is_end = False
        
        self.r = rospy.Rate(1)
        
        self.pub = rospy.Publisher('test', String, queue_size=1)
        self.end_pub = rospy.Publisher('end', Int32, queue_size=1)
        rospy.Subscriber('guess', Int32, self.callback, queue_size=1)


    
    def callback(self, data):
        if(data.data == self.number):
            self.is_end = True
            self.pub.publish("correct")
            self.end_pub.publish(-1)
        elif(data.data > self.number):
            self.pub.publish("Up")
        else:
            self.pub.publish("Down")
    
    def run(self):
        while not rospy.is_shutdown() and not self.is_end:
            pass
            
if __name__ == '__main__':
    game = NumberPublisher()
    game.run()
    

