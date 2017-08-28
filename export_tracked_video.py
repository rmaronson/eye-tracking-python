#!/usr/bin/env python

import sys
import numpy as np
import cv2
import os
import argparse
import itertools

import real_positions
import get_intrinsics
import metadata

class VideoExport:
    def __init__(self, filename):
        print 'Reading file', filename
        self.filename = filename
        video = cv2.VideoCapture(filename)
        if not video.isOpened():
            raise ValueError("Could not open video")
        
        self.video_info = (int(video.get(cv2.CAP_PROP_FOURCC)), video.get(cv2.CAP_PROP_FPS), int(video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        
        self.camera_matrix, self.distortion_coeffs = get_intrinsics.get_intrinsics(video=video)
                    
        self.points3d = np.expand_dims(real_positions.get_grid_points_3d(np.array([p for p in itertools.product(range(10), range(10))])), axis=0)
        self.points3dp1 = np.expand_dims(real_positions.get_grid_points_3d(
            np.concatenate([np.array([p for p in itertools.product(range(10), range(3))]),
                           np.array([p for p in itertools.product(range(5), range(3,10))])])
            
            ), axis=0)
        self.points3dp1[:,:,2] = self.points3dp1[:,:,2] + .05
        
        self.timestamps, self.robot_pos = metadata.load_metadata(filename)
        self.ref_frame, self.tfs = metadata.load_stabilization(filename)
        self.ext_cal = metadata.load_stable_extrinsics(filename)
        
        self.frames = dict()
        frame_id = 0
        print 'Loading frames...'
        while frame_id < len(self.timestamps):
            ok, frame = video.read()
            if not ok:
                break
            self.frames[frame_id] = frame
            frame_id = frame_id + 1
        print 'Done.'
        
    def get(self, frame_id, offset=0.):
        frame = np.array(self.frames[frame_id], copy=True)
        if self.ref_frame[frame_id] in self.ext_cal:
            color = (0,0,255)
            rvec = np.array(self.ext_cal[self.ref_frame[frame_id]]['rvec'])
            tvec = np.array(self.ext_cal[self.ref_frame[frame_id]]['tvec'])
            H = cv2.invertAffineTransform(self.tfs[frame_id])
            
            points2d_reprojected, _ = cv2.projectPoints(self.points3d, rvec, tvec, self.camera_matrix, self.distortion_coeffs)
            points2d_reprojected = cv2.transform(points2d_reprojected, H)
            points2d_int = np.array(np.squeeze(points2d_reprojected), dtype=int)
#             print points2d_int[0:3,:]
            for point in points2d_int:
                cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), color, 2)
                
            points2dp1, _ = cv2.projectPoints(self.points3dp1, rvec, tvec, self.camera_matrix, self.distortion_coeffs)
            points2dp1 = cv2.transform(points2dp1, H)
            points2dp1_int = np.array(np.squeeze(points2dp1), dtype=int)
#             print points2dp1_int[0:3,:]
            for point in points2dp1_int:
                cv2.rectangle(frame, tuple(point - [3,3]), tuple(point + [3,3]), (0,255,255), 2)
                
            robot3d = self.robot_pos.get_data(self.timestamps[frame_id]+offset)[None,:,:]
#             print robot3d
#                 print robot3d
            robot2d, _ = cv2.projectPoints(robot3d, rvec, tvec, self.camera_matrix, self.distortion_coeffs)
            robot2d = cv2.transform(robot2d, H)
            robot2d_int = np.array(np.squeeze(robot2d), dtype=int)
            for pt in robot2d_int:
#                     print pt
                try:
                    cv2.circle(frame, tuple(pt), 4, (0,255,0), -1)
                except OverflowError:
                    pass
            for pt1, pt2 in zip(robot2d_int, robot2d_int[1:,:]):
                try:
                    cv2.line(frame, tuple(pt1), tuple(pt2), (0,255,0), 2)
                except OverflowError:
                    pass
            
        return frame
    
    def adjust_time(self, init_offset=0.):
        idx = 0
        offset = init_offset
        step = 0.5
        play = False
        while True:
            frame = self.get(idx, offset)
            cv2.imshow("Frame", frame)
            
            if play:
                idx = idx+1
                if idx not in self.frames:
                    idx = idx-1
                    play = False
                    break
                key = cv2.waitKey(10)
            else:
                key = cv2.waitKey()
            if key == 2 and idx > 0: # left arrow
                idx = idx-1
            elif key == 3 and idx < len(self.frames): # right arrow
                idx = idx+1
            elif key == 119: # w
                step = step * 2
                print 'Step:', step
            elif key == 115: # s
                step = step / 2
                print 'Step:', step
            elif key == 97: # a
                offset = offset - step
                print 'Offset:', offset
            elif key == 100: # d
                offset = offset + step
                print 'Offset:', offset
            elif key == 27: # esc
                break
            elif key == 32: # space
                play = not play
        return offset
            
            
    def save(self, offset=0.):
        outfile = os.path.join(os.path.dirname(self.filename), '..', 'extrinsic_calibration', os.path.basename(self.filename))
        out_video = cv2.VideoWriter(outfile, self.video_info[0], self.video_info[1], self.video_info[2:], True)
        for idx in range(len(self.frames)):
            out_video.write(self.get(idx, offset))


if __name__ == '__main__' :
    
    parser = argparse.ArgumentParser('Extract tracked videos.')
    parser.add_argument('file')
    parser.add_argument('--cal', default='/Users/reubena/Box Sync/eyegaze_data_ada_eating_study/camera-calib.json', help='path to .json file with intrinsic calibration info')
    parser.add_argument('--timeshift', action='store_true')
    parser.add_argument('--export', action='store_true')
    args = parser.parse_args()
    
    v = VideoExport(args.file)
    offset = metadata.load_offset(args.file)
    if args.timeshift:
        offset = v.adjust_time(init_offset=offset)
        metadata.save_offset(args.file, offset)
    if args.export:
        v.save(offset=offset)
