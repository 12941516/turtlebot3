#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

#=======================<gaussianBlur>=======================#
def gaussianBlur(src):
    gaussian_src = cv2.GaussianBlur(src, (7,7), sigmaX=0, sigmaY=0)
    return src

#=======================<RL detection>=======================#
def RL_detection(src):
    src = gaussianBlur(src)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(
        src,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=200,
        param1=100,
        param2=60,
        minRadius=50,
        maxRadius=200
    )
    src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x,y,r) in circles[0,:]:
            cv2.circle(src, (x,y), r, (0,255,0), 2)
            cv2.circle(src, (x,y), 3, (0,0,255), -1)
            
            ###TODO : Implements codes in this block###
            
            ###########################################
            
    return src, 0

#======================<camera settings>=====================#
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("failed to open usb-camera(dev/video0)")
    exit()

#=====================<ROSnode settings>=====================#
rospy.init_node('turtlebot3_traffic_light')
twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

#===========================<main>===========================#
while not rospy.is_shutdown():
    ret, src = cap.read()
    dst, direction = RL_detection(src)
    
    twist = Twist()
    twist.linear.x = 0.0
    if direction == 1:
        twist.angular.z = 0.05
        rospy.loginfo("left_detected")
    elif direction == -1:
        twist.angular.z = -0.05
        rospy.loginfo("right_detected")
    else: twist.angular.z = 0.0
    
    cv2.imshow('src', src)
    cv2.imshow('dst', dst)
    if cv2.waitKey(1) == ord('q'):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        twist_pub.publish(twist)
        break

cap.release()
cv2.destroyAllWindows()
