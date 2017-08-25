#!/usr/bin/env python

import cv2
import sys
import detect_grid
import numpy as np
import real_positions
import yaml

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
     
    gridDetector = detect_grid.GridDetector()
     
    # Define an initial bounding box
#     bbox = (287, 23, 86, 320)
 
    # Uncomment the line below to select a different bounding box
#     bbox = cv2.selectROI("Select", frame, [], False)
 
    # Initialize tracker with first frame and bounding box
#     ok = tracker.init(frame, bbox)
    width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
    camera_matrix = np.array([ [width, 0, width/2], [0, height, height/2], [0, 0, 1] ], dtype=float)
    distortion_coeffs = np.zeros(4, dtype=float)
 
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        points2d, grid_vals, results = gridDetector.get_grid(frame)
        if points2d is not None:
            print points2d.shape
            points3d = real_positions.get_grid_points_3d(grid_vals)
            success, rvec, tvec, inliers = cv2.solvePnPRansac(np.expand_dims(points3d, axis=0), np.expand_dims(points2d, axis=0), camera_matrix, distortion_coeffs)
            if success:
                points2d_reprojected, _ = cv2.projectPoints(points3d, rvec, tvec, camera_matrix, distortion_coeffs)
                points2d_int = np.array(np.squeeze(points2d_reprojected), dtype=int)
                for point in points2d_int:
                    cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), (0,255,255), 2)
                
        print yaml.dump(results)
        # Display result
        cv2.imshow("Tracking", frame)
        cv2.waitKey()
 