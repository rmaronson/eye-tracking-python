#!/usr/bin/env python

import sys
import numpy as np
import cv2
import yaml
import os
import itertools

import detect_grid
import real_positions


if __name__ == '__main__' :
    
    if len(sys.argv) < 2:
        print "Usage: get_video_cal <video name>"
 
    filename = sys.argv[1]
    print 'Reading file', filename
    
    video = cv2.VideoCapture(filename)
 
    # Exit if video not opened.
    if not video.isOpened():
        print "Could not open video"
        sys.exit()
    
    filestem, _ = os.path.splitext(os.path.basename(filename))
    
    caldir = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', filestem)
     
    gridDetector = detect_grid.GridDetector()
    width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
    camera_matrix = np.array([ [width, 0, width/2], [0, height, height/2], [0, 0, 1] ], dtype=float)
    distortion_coeffs = np.zeros(4, dtype=float)
    
    
    outfile = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', os.path.basename(filename))
    out_video = cv2.VideoWriter(outfile, int(video.get(cv2.CAP_PROP_FOURCC)), video.get(cv2.CAP_PROP_FPS), (int(width), int(height)), True)
    points3d = np.expand_dims(real_positions.get_grid_points_3d(np.array([p for p in itertools.product(range(10), repeat=2)])), axis=0)
    
    frame_idx = 0
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
        
        calfile = os.path.join(caldir, 'ext_cal_%05d.yaml' % frame_idx)
        if os.path.exists(calfile):
            with open(calfile, 'r') as f:
                results = yaml.load(f)
            print 'Loaded cal for frame %d' % frame_idx, 'from', calfile
            
            if all(v in results for v in ['camera_matrix', 'distortion', 'rvec', 'tvec']):
                camera_matrix = np.array(results['camera_matrix'])
                distortion = np.array(results['distortion'])
                rvec = np.array(results['rvec'])
                tvec = np.array(results['tvec'])
                
                print '\tLoaded extrinsic data'
                points2d_reprojected, _ = cv2.projectPoints(points3d, rvec, tvec, camera_matrix, distortion_coeffs)
                points2d_int = np.array(np.squeeze(points2d_reprojected), dtype=int)
                for point in points2d_int:
                    cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), (0,255,255), 2)
        
        out_video.write(frame)
        
        frame_idx = frame_idx+1
        
