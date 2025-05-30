#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

#=======================<gaussianBlur>=======================#
def gaussianBlur(src):
    gaussian_src = cv2.GaussianBlur(src, (7,7), sigmaX=0, sigmaY=0)
    return src

#========================<hsv_inrange>=======================#
def hsv_inrange(src):
    lower_range = np.array([0,150,150], dtype=np.uint8)
    upper_range = np.array([15,255,255], dtype=np.uint8)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    src = cv2.inRange(src, lower_range, upper_range)
    return src

#========================<Morphology>========================#
def morphology(src):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    src = cv2.dilate(src, kernel)
    return src

#=================<ComponentsWithStatsFilter>================#
def componentsWithStatsFilter(src):
    min_area = 1000
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)
    valid_labels = np.where(stats[1:, cv2.CC_STAT_AREA] >= min_area)[0] + 1
    mask = np.isin(labels, valid_labels)
    filtered = (mask * 255).astype(np.uint8)
    return filtered

#=======================<stop_barrier>=======================#
def stop_barrier(src):
    ###TODO : Implements codes in this block###
    src = gaussianBlur(src)
    src = hsv_inrange(src)
    src = morphology(src)
    src = componentsWithStatsFilter(src)
    
    coordinates = []
    contours, hierachy = cv2.findContours(src, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    for contour in contours:
        if cv2.contourArea(contour) < 4000: continue
        m = cv2.moments(contour)
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])
        coordinates.append((cx, cy))
        cv2.circle(src, (cx, cy), 5, (255, 100, 100), -1)
    
    slope = []
    inf_count = 0
    if len(coordinates) >= 2:
        for i in range(len(coordinates) - 1):
            dx = coordinates[i][0] - coordinates[i+1][0]
            dy = coordinates[i][1] - coordinates[i+1][1]
            if dx == 0: inf_count += 1
            else: slope.append(abs(dy / dx))

        if inf_count > 0: return src, True
        if slope:
            slope_avg = sum(slope) / len(slope)
            if slope_avg > 1: return src, True
    ###########################################
    return src, False

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
    dst, detected = stop_barrier(src)
    
    twist = Twist()
    twist.angular.z = 0.0
    if detected == True:
        rospy.loginfo("Open")
        twist.linear.x = 0.0
    else: twist.linear.x = 0.05
    twist_pub.publish(twist)
    
    cv2.imshow('src', src)
    cv2.imshow('dst', dst)
    if cv2.waitKey(1) == ord('q'):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        twist_pub.publish(twist)
        break

cap.release()
cv2.destroyAllWindows()
