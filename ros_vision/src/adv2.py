#! /usr/bin/env python
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist

#=================<Sobel XY gradient Filter>=================#
def sobel_xy(src):
    src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    sobel_x = cv2.Sobel(src, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(src, cv2.CV_64F, 0, 1, ksize=3)
    gradmag = np.sqrt(sobel_x**2 + sobel_y**2)
    scale_factor = np.max(gradmag)/255  
    gradmag = (gradmag/scale_factor).astype(np.uint8)
    th_mag = (30, 255)  #(30, 255)
    gradient_magnitude = np.zeros_like(gradmag)
    gradient_magnitude[(gradmag >= th_mag[0]) & (gradmag <= th_mag[1])] = 255
    return gradient_magnitude

#=================<ComponentsWithStatsFilter>================#
def componentsWithStatsFilter(src):
    min_area = 50
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)
    valid_labels = np.where(stats[1:, cv2.CC_STAT_AREA] >= min_area)[0] + 1
    mask = np.isin(labels, valid_labels)
    filtered = (mask * 255).astype(np.uint8)
    return filtered

#================<Perspective Transformation>================#
def perspectiveTransformation(src):
    src_ptr = np.float32([[80,0],[560,0],[640,240],[0,240]]) # lu, ru, rd, ld
    dst_ptr = np.float32([[0,0],[640,0],[640,240],[0,240]])
    mtrx = cv2.getPerspectiveTransform(src_ptr, dst_ptr)
    src = cv2.warpPerspective(src, mtrx, (640, 240))
    return src
    
#========================<Morphology>========================#
def morphology(src):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    src = cv2.dilate(perspectiveTransformation(src), kernel)
    return src

#=======================<Curvaturate>========================#
def curvaturate(windows, window_h, src_width, src_height):
    window_count = len(windows)
    window_x = np.array(windows)
    window_y = np.array([src_height - window_h * (i + 0.5) for i in range(window_count)])
    valid_mask = (window_x > 0) & (window_x < src_width)
    valid_x = window_x[valid_mask]
    valid_y = window_y[valid_mask]
    if len(valid_x) < 3:
        return float('inf'), 0
    fit = np.polyfit(valid_y, valid_x, 2)
    y_eval = np.max(valid_y)
    A = fit[0]
    B = fit[1]
    curvature = ((1 + (2*A*y_eval+B) ** 2) ** 1.5) / np.abs(2*A)
    slope = 2*A*y_eval + B
    angle_rad = np.arctan(slope)
    angle_deg = -np.degrees(angle_rad)
    return curvature, angle_deg

#=====================<Compute Velocity>=====================#
def computeVelocity(angle_deg, curvature, max_angle=45.0, max_angular=1.0):
    max_linear = 0.22
    min_linear = 0.04
    min_curvature = 100
    angle = np.clip(angle_deg, -max_angle, max_angle)
    angular_z = (angle / max_angle) * max_angular
    if curvature < min_curvature:
        linear_x = min_linear
    else:
        factor = np.clip((curvature - min_curvature)/1000.0, 0.0, 1.0)
        linear_x = min_linear + (max_linear-min_linear) * factor
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist

#======================<Sliding Window>======================#
def slidingWindow(src):
    src = sobel_xy(src)
    src = componentsWithStatsFilter(src)
    src = morphology(src[src.shape[0]//2:, :])
    window_count = 10
    window_w = 140
    window_h = int(src.shape[0] / window_count)
    left_windows = []
    right_windows = []
    mid_windows = []
    for i in range(window_count): # 9~0 indexing
        left_sum, left_count, left_average = (0, 0, 0)
        right_sum, right_count, right_average = (0, 0, 0)
        contours, _ = cv2.findContours(src[src.shape[0]-window_h*(i+1):src.shape[0]-window_h*i,:], cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            if cv2.contourArea(contour) < 10: continue
            m = cv2.moments(contour)
            if m['m00'] == 0: continue
            cx = int(m['m10'] / m['m00'])
            if i == 0:
                if cx < src.shape[1] / 2:
                    left_sum += cx
                    left_count += 1
                else:
                    right_sum += cx
                    right_count += 1
            else:
                if (left_windows[i-1] - window_w/2) < cx < (left_windows[i-1] + window_w/2):
                    left_sum += cx
                    left_count += 1
                if (right_windows[i-1] - window_w/2) < cx < (right_windows[i-1] + window_w/2):
                    right_sum += cx
                    right_count += 1
        if left_count > 0:
            left_average = int(left_sum / left_count)
        else:
            left_average = left_windows[i-1] if i > 0 else 0
        if right_count > 0:
            right_average = int(right_sum / right_count)
        else:
            right_average = right_windows[i-1] if i > 0 else src.shape[1]
        left_windows.append(left_average)
        right_windows.append(right_average)
    for l, r in zip(left_windows, right_windows):
        if l==0 and r!=src.shape[1]:
            mid_windows.append(r-src.shape[1]//2)
        elif r==src.shape[1] and l!=0:
            mid_windows.append(l+src.shape[1]//2)
        elif 0 <l<src.shape[1] and 0<r<src.shape[1]:
            mid_windows.append((l+r)//2)
        else:
            mid_windows.append(src.shape[1]//2)
    
    src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    for i in range(window_count):
        cv2.rectangle(src, (int(left_windows[i]-window_w/2),int(src.shape[0]-window_h*(i+1))), (int(left_windows[i]+window_w/2),int(src.shape[0]-window_h*(i))), (255,255-20*i,255-20*i), 3)
        cv2.rectangle(src, (int(right_windows[i]-window_w/2),int(src.shape[0]-window_h*(i+1))), (int(right_windows[i]+window_w/2),int(src.shape[0]-window_h*(i))), (255-20*i,255-20*i,255), 3)
        cv2.circle(src, (int(mid_windows[i]),int(src.shape[0]-window_h*(i+0.5))), 3, (255-20*i,255,255-20*i), -1)
    curvature, angle = curvaturate(mid_windows, window_h, src.shape[1], src.shape[0])
    twist = computeVelocity(angle, curvature)
    return src, twist

#===========================<Main>===========================#
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("failed to open usb-camera(/dev/video0)")
    exit()

rospy.init_node('adv2_node')
twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
while not rospy.is_shutdown():
    ret, src = cap.read()
    result, twist = slidingWindow(src)
    twist_publisher.publish(twist)
    cv2.imshow('src', src)
    cv2.imshow('result', result)
    if cv2.waitKey(1) == ord('q'): break

#===========================<Quit>===========================#
cap.release()
cv2.destroyAllWindows()

