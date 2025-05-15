#! /usr/bin/env python
import numpy as np
import cv2 as c

def sobel_xy(src):
    src = c.cvtColor(src, c.COLOR_BGR2GRAY)
    sobel_x = c.Sobel(src, c.CV_64F, 1, 0, ksize=3)
    sobel_y = c.Sobel(src, c.CV_64F, 0, 1, ksize=3)
    g = np.sqrt(sobel_x**2 + sobel_y**2)
    s = np.max(g)/255
    g = (g/s).astype(np.uint8)
    th = (30, 255)
    gm = np.zeros_like(g)
    gm[(g >= th[0]) & (g <= th[1])] = 255
    return gm
    
def cpWS(src):
    min_area = 50
    nl, l, s, _ = c.connectedComponentsWithStats(src, connectivity=8)
    vl = np.where(s[1:, c.CC_STAT_AREA] >= min_area)[0] + 1
    mask = np.isin(l, vl)
    f = (mask*255).astype(np.uint8)
    return f
    
def perspectiveTransformation(src):
    src_ptr = np.float32([[80, 0], [560, 0], [640, 240], [0, 240]])
    dst_ptr = np.float32([[0, 0], [640, 0], [640, 240], [0, 240]])
    mtrx = c.getPerspectiveTransform(src_ptr, dst_ptr)
    src = c.warpPerspective(src, mtrx, (640, 240))
    return src

def morphology(src):
    kernel = c.getStructuringElement(c.MORPH_RECT, (9,9))
    src = c.dilate(perspectiveTransformation(src), kernel)
    return src

def slidingWindow(src):
    # video size = 640 * 240
    src = morphology(src[240:, :])
    window_count = 10
    window_w = 100
    window_h = int(src.shape[0] / window_count)
    left_windows = []
    right_windows = []

    for i in range(window_count):  # 9~0 indexing
        left_sum, left_count, left_average = (0, 0, 0)
        right_sum, right_count, right_average = (0, 0, 0)
        contours, _ = c.findContours(src[src.shape[0]-window_h*(i+1):src.shape[0]-window_h*i, :], c.RETR_CCOMP, c.CHAIN_APPROX_NONE)
        for contour in contours:
            if(c.contourArea(contour) < 50): continue
            m = c.moments(contour)
            cx = int(m['m10'] / m['m00'])

            if(not i):
                if(cx < src.shape[1] / 2):
                    left_sum += cx
                    left_count += 1
                else:
                    right_sum += cx
                    right_count += 1
            else:
                if(cx < left_windows[i-1] + window_w / 2 and cx > left_windows[i-1] - window_w / 2):
                    left_sum += cx
                    left_count += 1
                if(cx < right_windows[i-1] + window_w / 2 and cx > right_windows[i-1] - window_w / 2):
                    right_sum += cx
                    right_count += 1

        try:
            left_average = int(left_sum / left_count)
        except:
            pass
        try:
            right_average = int(right_sum / right_count)
        except:
            pass

        left_windows.append(left_average)
        right_windows.append(right_average)

    for i in range(window_count):
        if(i == 0):
            if(left_windows[i] == 0):
                left_windows[i] = 0
            if(right_windows[i] == 0):
                right_windows[i] = src.shape[1]
        elif(i == window_count - 1):
            if(left_windows[i] == 0):
                left_windows[i] = 0
            if(right_windows[i] == 0):
                right_windows[i] = src.shape[1]
        else:
            if(left_windows[i] == 0):
                left_windows[i] = 0
            if(right_windows[i] == 0):
                right_windows[i] = src.shape[1]

        c.rectangle(src, (int(left_windows[i] - window_w / 2), int(src.shape[0] - window_h * (i + 1))),
                    (int(left_windows[i] + window_w / 2), int(src.shape[0] - window_h * i)), 
                    (100 + 10 * i, 100 + 10 * i, 100 + 10 * i), 3)
        c.rectangle(src, (int(right_windows[i] - window_w / 2), int(src.shape[0] - window_h * (i + 1))),
                    (int(right_windows[i] + window_w / 2), int(src.shape[0] - window_h * i)), 
                    (100 + 10 * i, 100 + 10 * i, 100 + 10 * i), 3)

    return src, left_windows, right_windows

cap = c.VideoCapture(0)
cap.set(c.CAP_PROP_FRAME_WIDTH, 640)
cap.set(c.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("failed to open usb-camera(/dev/video0)")
    exit()

while True:
    ret, src = cap.read()
    result, left_windows, right_windows = slidingWindow(cpWS(sobel_xy(src)))
    c.flip(src, 1)
    c.flip(result, 1)
    c.imshow('src', src)
    c.imshow('result', result)
    if c.waitKey(1) == ord('q'): break

cap.release()
c.destroyAllWindows()
