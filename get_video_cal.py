#!/usr/bin/env python

import sys
import numpy as np
import cv2
import yaml
import os
import get_intrinsics

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
    
    outdir = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', filestem)
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    print 'Saving resulting files to', outdir

    gridDetector = detect_grid.GridDetector()
    frame_idx = 0
    camera_matrix, distortion_coeffs = get_intrinsics.get_intrinsics(video=video)

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
                results['camera_matrix'] = camera_matrix.tolist()
                results['distortion'] = distortion_coeffs.tolist()
                results['rvec'] = rvec.tolist()
                results['tvec'] = tvec.tolist()
                
                points2d_reprojected, _ = cv2.projectPoints(points3d, rvec, tvec, camera_matrix, distortion_coeffs)
                points2d_int = np.array(np.squeeze(points2d_reprojected), dtype=int)
                for point in points2d_int:
                    cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), (0,255,255), 2)
        
        
        outfile = os.path.join(outdir, 'ext_cal_%05d.yaml' % frame_idx)
        with open(outfile, 'w') as f:
            yaml.dump(results, f)
        print 'Saved frame %d' % frame_idx, 'to', outfile
        
        
        frame_idx = frame_idx+1
        
