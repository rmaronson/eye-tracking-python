#!/usr/bin/env python

import cv2
import os
import yaml
import numpy as np
import argparse
import multiprocessing
import itertools
from timeit import default_timer as timer

import metadata
import real_positions
import get_intrinsics



def build_all_extrinsics(args):#videofile, intrinsics):
    videofile, intrinsics = args
    camera_matrix, distortion_coeffs = intrinsics
    ref_frame, tfs = metadata.load_stabilization(videofile)
    root_dir = os.path.dirname(videofile)
    trial_name, _ = os.path.splitext(os.path.basename(videofile))
    cal_dir = os.path.join(root_dir, '..', 'extrinsic_calibration', trial_name)
    
    frame_id = 0
    filename = os.path.join(cal_dir, 'ext_cal_%05d.yaml') % frame_id
    
    cur_ref = -1
    cur_2d = []
    cur_grid = []
    
    cal_data = dict()
    print 'Started file:', videofile
    while (os.path.exists(filename)):
#         print 'Reading file: ', filename
        with open(filename, 'r') as infile:
            data = yaml.load(infile)
            
            if ref_frame[frame_id] != cur_ref:
                # Finish the previos step, if appropriate
                if len(cur_2d) > 2:
                    # Dump the parameters to file
                    p2_file = os.path.join(cal_dir, 'cal_points_%05d_2d.npy' % cur_ref)
                    grid_file = os.path.join(cal_dir, 'cal_points_%05d_grid.npy' % cur_ref)
#                     print 'Saving data for', cur_ref, 'to', p2_file, ',', p3_file
                    np.save(p2_file, cur_2d)
                    np.save(grid_file, cur_grid)
                    cur_3d = real_positions.get_grid_points_3d(cur_grid)
                    print 'Running solvePnP on', cur_3d.shape[0], 'values'
                    success, rvec, tvec, inliers = cv2.solvePnPRansac(cur_3d[None,:,:], cur_2d[None,:,:], camera_matrix, distortion_coeffs, iterationsCount=max(100, cur_3d.shape[0]))
                    if success:
                        cal_data[cur_ref] = { 'rvec': rvec.tolist(), 'tvec': tvec.tolist(), 'inliers': inliers.tolist() }
                # Reset the data
                cur_ref = ref_frame[frame_id]
                cur_2d = np.empty([0,2])
                cur_grid = np.empty([0,2])
#                 cur_3d = np.empty([0,3])
                    
            if all(key in data for key in ['grid_vals', 'points2d']):
                # Append the latest data
                points2d = np.array(data['points2d'])
#                 print tfs[frame_id]
                H = tfs[frame_id] #cv2.invertAffineTransform( tfs[frame_id] ) # Invert because Matlab's affine2d implicitly inverts
                points2d_stab = cv2.transform(points2d[None,:,:], H)
#                 points3d = real_positions.get_grid_points_3d(np.array(data['grid_vals']))
                cur_2d = np.concatenate([cur_2d, points2d_stab[0,:,:]])
                cur_grid = np.concatenate([cur_grid, np.array(data['grid_vals'])])
        
        frame_id = frame_id+1
        filename = os.path.join(cal_dir, 'ext_cal_%05d.yaml') % frame_id
        
    outfile = os.path.join(cal_dir, 'all_stable_extrinsics.yaml')
    with open(outfile, 'wb') as f:
#         writer = csv.DictWriter(f, ['ref_frame', 'rvec', 'tvec', 'inliers'])
#         writer.writeheader()
#         map(writer.writerow, cal_data)
        yaml.dump(cal_data, f)
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser('Extract extrinsic calibration information from preprocessed videos.')
    parser.add_argument('files', nargs='+')
    parser.add_argument('--cal', default='/Users/reubena/Box Sync/eyegaze_data_ada_eating_study/camera-calib.json', help='path to .json file with intrinsic calibration info')
    parser.add_argument('--pool', type=int, default=None, help='number of processes to use (default=cpu_cores())')
    args = parser.parse_args()
    intrinsics = get_intrinsics.get_intrinsics(filename=args.cal)
    if args.pool == 1:
        map(lambda x: build_all_extrinsics((x, intrinsics)), args.files)
    else:
        pool = multiprocessing.Pool(args.pool)
        multiprocessing.log_to_stderr()
        start = timer()
        pool.map(build_all_extrinsics, itertools.izip_longest(args.files, [], fillvalue=intrinsics))
        end = timer()
        print 'Finished', len(args.files), 'files in', (end-start), 'sec. (Avg: ', ((end-start)/len(args.files)), ' per file)'
    
