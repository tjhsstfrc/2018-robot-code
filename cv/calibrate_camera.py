import numpy as np
import cv2
from pyimagesearch.shapedetector import ShapeDetector
import imutils

cap = cv2.VideoCapture(0)

mouseX, mouseY = 0, 0

totalColors = []
totalLocations = []

global frame

def add_loc(event, x, y, flags, param):
    if event == cv2.EVENT_FLAG_LBUTTON:
        totalLocations.append((x,y))
        add_color((x,y))

def add_color(tup):
    totalColors.append(list([frame[tup[1]][tup[0]][0], frame[tup[1]][tup[0]][1], (frame[tup[1]][tup[0]][2])]))

def getRanges(colors, ind=0):
    l = []
    for each in colors:
        l.append(each[ind])
    l = sorted(l)
    return (l[0], l[::-1][0])

def printRanges():
    low = []
    high = []
    x = getRanges(totalColors, ind=0)    
    y = getRanges(totalColors, ind=1)    
    z = getRanges(totalColors, ind=2)    
    low = [x[0], y[0], z[0]]
    high = [x[1], y[1], z[1]]
    print(low)
    print(high)

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', add_loc)

while(True):
    # Capture frame-by-frame
    ret,frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1) & 0xFF
    
    if k == ord('q'):
        printRanges()
        break
    
