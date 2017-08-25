#!/usr/bin/env python

import json
import os
import cv2
import numpy as np

def get_intrinsics(filename="/Users/reubena/Box Sync/eyegaze_data_ada_eating_study/camera-calib.json", video=None):
    print 'Reading file', filename
    camera_matrix = None
    if os.path.exists(filename):
        print 'Reading file', filename
        with open(filename, 'r') as f:
            jsondata = json.load(f)
        if "intrinsics_opencv" in jsondata and [k in jsondata["intrinsics_opencv"] for k in ["camera_matrix", "distortion_coeffs"]]:
            camera_matrix = np.array(jsondata["intrinsics_opencv"]["camera_matrix"])
            distortion_coeffs = np.array(jsondata["intrinsics_opencv"]["distortion_coeffs"])
            
    
    if camera_matrix is None:
        width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
        camera_matrix = np.array([ [width, 0, width/2], [0, height, height/2], [0, 0, 1] ], dtype=float)
        distortion_coeffs = np.zeros(4, dtype=float)
        
    return camera_matrix, distortion_coeffs
