#!/usr/bin/env python

import sys
import numpy as np
import cv2
import os
import itertools
import csv

import detect_grid
import real_positions
import get_intrinsics


if __name__ == '__main__' :
    
    if len(sys.argv) < 2:
        print "Usage: get_video_cal <video name>"
        sys.exit(1)
 
    filename = sys.argv[1]
    print 'Reading file', filename
    
    video = cv2.VideoCapture(filename)
 
    # Exit if video not opened.
    if not video.isOpened():
        print "Could not open video"
        sys.exit()
    
    filestem, _ = os.path.splitext(os.path.basename(filename))
    
    calfile = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', filestem, 'all_extrinsics_filtered.csv')
     
    gridDetector = detect_grid.GridDetector()
    width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
    camera_matrix, distortion_coeffs = get_intrinsics.get_intrinsics(video=video)
    
    outfile = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', os.path.basename(filename))
    out_video = cv2.VideoWriter(outfile, int(video.get(cv2.CAP_PROP_FOURCC)), video.get(cv2.CAP_PROP_FPS), (int(width), int(height)), True)
    points3d = np.expand_dims(real_positions.get_grid_points_3d(np.array([p for p in itertools.product(range(10), repeat=2)])), axis=0)
    
    frame_idx = 0
    with open(calfile, 'r') as cal:
        print 'Read calibration file: ', calfile
        cal_reader = csv.DictReader(cal)
        cal_info = cal_reader.next()
        while True:
            # Read a new frame
            ok, frame = video.read()
            if not ok:
                break
            
            print 'Projecting', cal_info
            print frame_idx
            if int(cal_info['frame_id']) == frame_idx:
                color = (0,255,255) if int(cal_info['ok']) else (0,0,255)
                rvec = np.array([[ float(cal_info[k]) for k in ('r1','r2','r3') ]])
                tvec = np.array([[ float(cal_info[k]) for k in ('x','y','z') ]])
                
                points2d_reprojected, _ = cv2.projectPoints(points3d, rvec, tvec, camera_matrix, distortion_coeffs)
                points2d_int = np.array(np.squeeze(points2d_reprojected), dtype=int)
                for point in points2d_int:
                    cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), color, 2)
                cal_info = cal_reader.next()
            
            out_video.write(frame)
            
            frame_idx = frame_idx+1
        
