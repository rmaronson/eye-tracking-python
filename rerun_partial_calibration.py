#!/usr/bin/env python

import shutil
import os
import yaml
import numpy as np
import cv2
import sys

import real_positions

def rerun_3d_cal(root_dir):
    frame_id = 0
    filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id
    archive_id = 0
    while os.path.exists(os.path.join(root_dir, 'archive%02d' % archive_id)):
        archive_id = archive_id + 1
    archive_dir = os.path.join(root_dir, 'archive%02d' % archive_id)
    os.makedirs(archive_dir)
    
    while (os.path.exists(filename)):
        results = {}
        print filename
        with open(filename, 'r') as infile:
            results = yaml.load(infile)
#                 results['camera_matrix'] = camera_matrix.tolist()
#                 results['distortion'] = distortion_coeffs.tolist()
#                 results['rvec'] = rvec.tolist()
#                 results['tvec'] = tvec.tolist()
#                 results['ransac_inliers'] = inliers.tolist()
        if 'rvec' in results:
            camera_matrix = np.array(results['camera_matrix'])
            distortion_coeffs = np.array(results['distortion'])
            points2d = np.array(results['points2d'])
            points3d = real_positions.get_grid_points_3d(np.array(results['grid_vals']))
            success, rvec, tvec, inliers = cv2.solvePnPRansac(points3d[None,:,:], points2d[None,:,:], camera_matrix, distortion_coeffs)
            print 'Recalculated for frame', frame_id
            if success:
                print results['tvec'], '==>', tvec
                print results['rvec'], '==>', rvec
                results['rvec'] = rvec.tolist()
                results['tvec'] = tvec.tolist()
                results['ransac_inliers'] = inliers.tolist()
            else:
                del(results['rvec'] )
                del(results['tvec'] )
                del(results['ransac_inliers'] )
            
            shutil.move(filename, os.path.join(archive_dir, os.path.basename(filename)))
            with open(filename, 'w') as outfile:
                yaml.dump(results, outfile)
            
        frame_id = frame_id+1
        filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id
                
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'Usage: rerun_partial_calibration <directory(ies)>'
        sys.exit(1)
        
    map(rerun_3d_cal, sys.argv[1:])
