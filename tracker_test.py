#!/usr/bin/env python

import cv2
import sys
import detect_grid
import numpy as np


if __name__ == '__main__' :
 
    # Set up tracker.
    # Instead of MIL, you can also use
    # BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN
 
    # Read video
    video = cv2.VideoCapture("../p10/videos/p10_blend1.mp4")
 
    # Exit if video not opened.
    if not video.isOpened():
        print "Could not open video"
        sys.exit()
 
    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print 'Cannot read video file'
        sys.exit()
     
    gridDetector = detect_grid.GridDetector()
     
    # Define an initial bounding box
#     bbox = (287, 23, 86, 320)
 
    # Uncomment the line below to select a different bounding box
#     bbox = cv2.selectROI("Select", frame, [], False)
 
    # Initialize tracker with first frame and bounding box
#     ok = tracker.init(frame, bbox)
 
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        gridDetector.get_grid(frame)
        
        # Display result
        cv2.imshow("Tracking", frame)
        cv2.waitKey(1000)
 