#!/usr/bin/env python

import sys
import cv2
import os
import yaml
import numpy as np
import csv

import load_video_metadata
import real_positions
import get_intrinsics



def build_all_extrinsics(videofile, intrinsics):
    camera_matrix, distortion_coeffs = intrinsics
    ref_frame, tfs = load_video_metadata.load_stabilization(videofile)
    root_dir = os.path.dirname(videofile)
    trial_name, _ = os.path.splitext(os.path.basename(videofile))
    cal_dir = os.path.join(root_dir, '..', 'extrinsic_calibration', trial_name)
    
    frame_id = 0
    filename = os.path.join(cal_dir, 'ext_cal_%05d.yaml') % frame_id
    
    cur_ref = -1
    cur_2d = []
    cur_3d = []
    
    cal_data = dict()
    
    while (os.path.exists(filename)):
        print 'Reading file: ', filename
        with open(filename, 'r') as infile:
            data = yaml.load(infile)
            
            if ref_frame[frame_id] != cur_ref:
                # Finish the previos step, if appropriate
                if len(cur_2d) > 2:
                    print 'Running solvePnP on', cur_3d.shape
                    success, rvec, tvec, inliers = cv2.solvePnPRansac(cur_3d[None,:,:], cur_2d[None,:,:], camera_matrix, distortion_coeffs, iterationsCount=cur_3d.shape[0])
                    if success:
                        cal_data[cur_ref] = { 'rvec': rvec.tolist(), 'tvec': tvec.tolist(), 'inliers': inliers.tolist() }
                # Reset the data
                cur_ref = ref_frame[frame_id]
                cur_2d = np.empty([0,2])
                cur_3d = np.empty([0,3])
                    
            if all(key in data for key in ['grid_vals', 'points2d']):
                # Append the latest data
                points2d = np.array(data['points2d'])
                print tfs[frame_id]
                H = tfs[frame_id] #cv2.invertAffineTransform( tfs[frame_id] ) # Invert because Matlab's affine2d implicitly inverts
                points2d_stab = cv2.transform(points2d[None,:,:], H)
                points3d = real_positions.get_grid_points_3d(np.array(data['grid_vals']))
                cur_2d = np.concatenate([cur_2d, points2d_stab[0,:,:]])
                cur_3d = np.concatenate([cur_3d, points3d])
        
        frame_id = frame_id+1
        filename = os.path.join(cal_dir, 'ext_cal_%05d.yaml') % frame_id
        
    outfile = os.path.join(cal_dir, 'all_stable_extrinsics.yaml')
    with open(outfile, 'wb') as f:
#         writer = csv.DictWriter(f, ['ref_frame', 'rvec', 'tvec', 'inliers'])
#         writer.writeheader()
#         map(writer.writerow, cal_data)
        yaml.dump(cal_data, f)
        
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'Usage: build_grouped_stable_cal <videofiles>'
        sys.exit(1)
    intrinsics = get_intrinsics.get_intrinsics()
    for f in sys.argv[1:]:
        build_all_extrinsics(f, intrinsics)
    
