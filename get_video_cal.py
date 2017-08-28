#!/usr/bin/env python

import numpy as np
import cv2
import yaml
import os
import errno
import argparse
import multiprocessing
import itertools
from timeit import default_timer as timer

import detect_grid
import real_positions
import get_intrinsics


def get_video_cal(info): #filename, intrinsics):
    filename, intrinsics = info
    print 'Reading file', filename
    
    video = cv2.VideoCapture(filename)
 
    # Exit if video not opened.
    if not video.isOpened():
        print "Could not open video"
        raise ValueError("No file available: " + filename)
    
    filestem, _ = os.path.splitext(os.path.basename(filename))
    
    outdir = os.path.join(os.path.dirname(filename), '..', 'extrinsic_calibration', filestem)
    if not os.path.exists(outdir):
        try:
            os.makedirs(outdir)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
#     print 'Saving resulting files to', outdir

    gridDetector = detect_grid.GridDetector()
    frame_idx = 0
    camera_matrix, distortion_coeffs = intrinsics

    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        points2d, grid_vals, results = gridDetector.get_grid(frame)
#         if points2d is not None:
#             points3d = real_positions.get_grid_points_3d(grid_vals)
#             success, rvec, tvec, inliers = cv2.solvePnPRansac(np.expand_dims(points3d, axis=0), np.expand_dims(points2d, axis=0), camera_matrix, distortion_coeffs)
#             if success:
#                 results['camera_matrix'] = camera_matrix.tolist()
#                 results['distortion'] = distortion_coeffs.tolist()
#                 results['rvec'] = rvec.tolist()
#                 results['tvec'] = tvec.tolist()
#                 results['ransac_inliers'] = inliers.tolist()
        
        outfile = os.path.join(outdir, 'ext_cal_%05d.yaml' % frame_idx)
        with open(outfile, 'w') as f:
            yaml.dump(results, f)
        if frame_idx % 100 == 0:
            print 'Saved', filename, ' frame %d' % frame_idx
        frame_idx = frame_idx+1

if __name__ == "__main__":
    parser = argparse.ArgumentParser('Extract extrinsic calibration information from videos.')
    parser.add_argument('files', nargs='+')
    parser.add_argument('--cal', default='/Users/reubena/Box Sync/eyegaze_data_ada_eating_study/camera-calib.json', help='path to .json file with intrinsic calibration info')
    parser.add_argument('--pool', type=int, default=None, help='number of processes to use (default=cpu_cores())')
    args = parser.parse_args()
    
    intrinsics = get_intrinsics.get_intrinsics(filename=args.cal)
    if args.pool == 1:
        map(lambda x: get_video_cal((x, intrinsics)), args.files)
    else:
        pool = multiprocessing.Pool(args.pool)
        multiprocessing.log_to_stderr()
        start = timer()
        pool.map(get_video_cal, itertools.izip_longest(args.files, [], fillvalue=intrinsics))
        end = timer()
        print 'Finished', len(args.files), 'files in', (end-start), 'sec. (Avg: ', ((end-start)/len(args.files)), ' per file)'
        
    
    
        
