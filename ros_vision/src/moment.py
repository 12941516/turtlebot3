#! /usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("failed to open usb-camera(dev/video0")
    exit()

lower_range = np.array([0,0,150], dtype=np.uint8)
upper_range = np.array([15,255,255], dtype=np.uint8)

while True:
    ret, src = cap.read()
    hsv_src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hsv_dst = cv2.inRange(hsv_src, lower_range, upper_range)

    contours, hierachy = cv2.findContours(hsv_dst, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        if(cv2.contourArea(contour) < 4000): continue
        m = cv2.moments(contour)

        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])
        cv2.circle(src, (cx, cy), 5, (255, 100, 100), -1)
    cv2.imshow('src', src)
    cv2.imshow('hsv_dst', hsv_dst)
    if cv2.waitKey(1) == ord('q'): break

cap.release()
cv2.destroyAllWindows()
