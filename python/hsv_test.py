from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
max_value = 255
max_value_H = 360//2
lower_border = 0, 0, 0
upper_border = max_value_H,max_value,max_value
low_H2 = 0
high_H2 = max_value_H
upper_border = max_value_H,max_value,max_value

#hier Startwerte eingeben

lower_border = 0, 0, 0
upper_border = 0, 0, 0
low_H2 = 0
high_H2 = 0

low_H, low_S, low_V = lower_border
high_H, high_S, high_V = upper_border

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name2 = 'Low H2'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name2 = 'High H2'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
def on_low_H_thresh_trackbar2(val):
    global low_H2
    global high_H2
    low_H2 = val
    low_H2 = min(high_H2-1, low_H2)
    cv.setTrackbarPos(low_H_name2, window_detection_name, low_H2)
def on_high_H_thresh_trackbar2(val):
    global low_H2
    global high_H2
    high_H2 = val
    high_H2 = max(high_H2, low_H2+1)
    cv.setTrackbarPos(high_H_name2, window_detection_name, high_H2)
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)
parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv.VideoCapture(args.camera)
#cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(low_H_name2, window_detection_name , low_H2, max_value_H, on_low_H_thresh_trackbar2)
cv.createTrackbar(high_H_name2, window_detection_name , high_H2, max_value_H, on_high_H_thresh_trackbar2)
cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
def creating_roi(frame):
    """Gibt die untere Bildhälfte zurück

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: untere Bildhälfte
    """    
    height, width, _ = frame.shape
    roi = frame[int(1/2*height):height,0:width]
    return roi
while True:
    
    ret, frame = cap.read()
    if frame is None:
        break
    frame = cv.rotate(frame, cv.ROTATE_180)
    roi = creating_roi(frame)
    frame_HSV = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
    frame_thresholda = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    frame_thresholdb = cv.inRange(frame_HSV, (low_H2, low_S, low_V), (high_H2, high_S, high_V))
    frame_threshold = cv.addWeighted(frame_thresholda,1,frame_thresholdb,1,0)

    #cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)
    
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
