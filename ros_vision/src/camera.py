#! /usr/bin/env python
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("failed to open usb-camera(dev/video0")
    exit()

while True:
    ret, src = cap.read()
    cv2.imshow('src', src)
    if cv2.waitKey(1) == ord('q'): break

cap.release()
cv2.destroyAllWindows()
