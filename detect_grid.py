#!/usr/bin/env python

import cv2
import numpy as np
from shapely.geometry import LineString, LinearRing
import matplotlib.pyplot as plt



def get_line_pairs(lines):
    def get_line_pair(line):
        a = np.cos(line[0][1])
        b = np.sin(line[0][1]);
        x0 = a*line[0][0]
        y0 = b*line[0][0];
        return ((x0 + 1000*(-b), y0 + 1000*(a)), (x0 - 1000*(-b), y0 - 1000*(a)))
    return [get_line_pair(l) for l in lines]


class GridDetector:
    def __init__(self):
        self.point_detector = GridPointDetector()

    def get_grid(self, frame):
        blobs = self.point_detector.detect(frame)
        self.blobs = blobs
        
        im_keypoints_only = np.zeros(frame.shape, dtype=frame.dtype)
    
        for key in blobs:
            int_pt = tuple([int(x) for x in key.pt])
            im_keypoints_only = cv2.circle(im_keypoints_only, int_pt, 0, (255,255,255), -1)

    
        lines = cv2.HoughLines(cv2.cvtColor(im_keypoints_only, cv2.COLOR_BGR2GRAY), 1, np.pi/180, 3, 40)
        self.lines = lines
        
        if lines is not None and len(lines) > 4:
#             lines = get_line_pairs(lines)
            H, horz_idx, vert_idx = get_rectification_ransac(np.array(lines), im_keypoints_only)
            
            if H is not None:
                rect_frame = cv2.warpPerspective(frame, H, frame.shape[1::-1])
                cv2.imshow("Rectified image", rect_frame)
        

class GridPointDetector:
    def __init__(self):
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 100
        params.thresholdStep = 5
        
        params.filterByArea = True
        params.maxArea = 80
        
        params.filterByConvexity = True
        params.minConvexity = 0.8
        
        self.detector = cv2.SimpleBlobDetector_create(params)
        
    def detect(self, frame):
        return self.detector.detect(frame)

def get_rectification_ransac(line_hough, frame=[], N=500, tol=0.05):
    """
    lines = list of tuples (rho, theta) of lines
    """
    if len(line_hough) < 4:
        return None, None, None
    lines = get_line_pairs(line_hough)
    line_pairs = np.array(lines, dtype=np.float32)
    lines_first = np.array([line_pairs[:,0,:]])
    lines_second = np.array([line_pairs[:,1,:]])
    
    lines_g1_b = np.abs(line_hough[:,0,1] - np.pi/2) < np.pi/8
    lines_g1 = np.flatnonzero(lines_g1_b)
    lines_g2_b = np.abs(line_hough[:,0,1] - np.pi) < np.pi/4
    lines_g2 = np.flatnonzero(lines_g2_b)
    
    if lines_g1.size < 2 or lines_g2.size < 2:
        return None, None, None
    
#     plt.scatter(line_hough[np.logical_not(np.logical_or(lines_g1_b, lines_g2_b)),0,0], line_hough[np.logical_not(np.logical_or(lines_g1_b, lines_g2_b)),0,1], c='b')
#     plt.scatter(line_hough[lines_g1,0,0], line_hough[lines_g1,0,1], c='r')
#     plt.scatter(line_hough[lines_g2,0,0], line_hough[lines_g2,0,1], c='g')
#     plt.show()
    
    rect_pts = np.array(((0,0), (100,0), (100,100), (0,100)), dtype=np.float32) + 100
    horz_idx = []
    vert_idx = []
    uniq = 0
    H = []
    
    animate = len(frame) > 0
        
    for i in range(N):
#         ran_lines = np.random.choice(range(len(line_pairs)), 4, replace=False)
        ran_g1 = np.random.choice(lines_g1, 2, replace=False)
        ran_g2 = np.random.choice(lines_g2, 2, replace=False)
        ran_lines = np.concatenate([ran_g1, ran_g2])
        
        
        ran_strings = [LineString(line_pairs[r]) for r in ran_lines]
        pts_geom = [
            ran_strings[0].intersection(ran_strings[2+0]),
            ran_strings[1].intersection(ran_strings[2+0]),
            ran_strings[1].intersection(ran_strings[2+1]),
            ran_strings[0].intersection(ran_strings[2+1])
        ]
        try:
            pts = np.array([(pt.x, pt.y) for pt in pts_geom], dtype=np.float32)
        except AttributeError:
            # one of them is not a point
            continue
                
        # Avoid self-intersections
        ring = LinearRing(pts)
        if not ring.is_simple:
            continue
        skip = False
        for j in range(len(pts_geom)):
            if pts_geom[j].distance(pts_geom[(j+1) % len(pts_geom)]) < 10:
                skip = True
        if skip:
            continue
        if not ring.is_ccw:
            pts = pts[::-1]
        
        if animate:
            cur_frame = np.array(frame, copy=True)
            for j in range(len(line_pairs)):
                color = (255,0,0)
                if j in ran_lines[0:2]:
                    color = (0,0,255)
                elif j in ran_lines[2:]:
                    color = (0,255,255)
                cv2.line(cur_frame, tuple(lines_first[0,j,:]), tuple(lines_second[0,j,:]), color)
            cv2.circle(cur_frame, tuple(pts[0]), 3, (255,0,0))
            cv2.circle(cur_frame, tuple(pts[1]), 3, (255,0,0))
            cv2.circle(cur_frame, tuple(pts[2]), 3, (255,0,0))
            cv2.circle(cur_frame, tuple(pts[3]), 3, (255,0,0))
            cv2.imshow("Selected lines", cur_frame)
        
        cur_H = cv2.getPerspectiveTransform(pts, rect_pts)
        lines1 = np.squeeze(cv2.perspectiveTransform(lines_first, cur_H))
        lines2 = np.squeeze(cv2.perspectiveTransform(lines_second, cur_H))
        cur_horz_idx = []
        cur_vert_idx = []        
        
        for j in range(len(lines1)):
            dx = lines1[j][0] - lines2[j][0]
            dy = lines1[j][1] - lines2[j][1]
            if abs(dy/dx) < tol:
                cur_horz_idx.append(j)
            elif abs(dx/dy) < tol:
                cur_vert_idx.append(j)

#         xs = line_pairs[cur_vert_idx, :, 0].ravel()
#         ys = line_pairs[cur_horz_idx, :, 1].ravel()
#         width = np.max(xs) - np.min(xs)
#         height = np.max(ys) - np.min(ys)

        def get_unique_lines(idx, lines1, lines2):
            uniq = []
            uniq_idx = []
            for j in range(len(idx)):
                big_line = LineString([lines1[j], lines2[j]]).buffer(1)
                add = True
                for uniq_line in uniq:
                    if big_line.intersects(uniq_line):
                        add = False
                        break
                if add:
                    uniq.append(big_line)
                    uniq_idx.append(j)
            return uniq_idx
        
        uniq_horz_lines = get_unique_lines(cur_horz_idx, lines1, lines2)
        uniq_vert_lines = get_unique_lines(cur_vert_idx, lines1, lines2)
                    
        if len(uniq_horz_lines) + len(uniq_vert_lines) > uniq:
            horz_idx = cur_horz_idx
            vert_idx = cur_vert_idx
            H = cur_H
            uniq = len(uniq_horz_lines) + len(uniq_vert_lines)
            print 'Update: ', H, ' horz=', len(horz_idx), ' vert=', len(vert_idx), ' uniq=', uniq
            if animate:
                rect_frame = np.zeros(cur_frame.shape, dtype=cur_frame.dtype)
                for j in range(len(lines1)):
                    color = (255,0,0)
                    if j in ran_lines[0:2]:
                        color = (0,0,255)
                    elif j in ran_lines[2:]:
                        color = (0,255,255)
                    elif j in cur_horz_idx or j in cur_vert_idx:
                        color = (255,255,0)
                    cv2.line(rect_frame, tuple(lines1[j,:]), tuple(lines2[j,:]), color)
                cv2.imshow("Rectified shape", rect_frame)
                cv2.waitKey()
            
    
    return H, horz_idx, vert_idx
            
            
        
        
        
# see https://stackoverflow.com/a/479028/2170381
# def rationalizations(x):
#     assert 0 <= x
#     ix = int(x)
#     yield ix, 1
#     if x == ix: return
#     for numer, denom in rationalizations(1.0/(x-ix)):
#         yield denom + ix * numer, numer
#     
# def rational_approx(a, tol=0.01):
#     ar_gen = rationalizations(a)
#     ar = ar_gen.next()
#     while abs(a - float(ar[0])/ar[1]) > tol:
#         ar = ar_gen.next()
#     return ar
#     
# def gcd_approx(a, b, mult=1000, tol=0.01):
#     print a, b
#     a = abs(a)
#     b = abs(b)
#     ar = rational_approx(a, tol)
#     br = rational_approx(b, tol)
#     return float(fractions.gcd(ar[0]*br[1], ar[1]*br[0])) / (ar[1]*br[1])

# def gcd_approx(a, b, tol=0.01):
#     if b > a:
#         a, b = b, a
#     while b > tol:
#         a, b = b, a%b
#     return a
# 
# def gcd_get_inliers(vals, gcd, tol=0.01):
#     div = vals / gcd
#     intval = np.round(div)
#     rem = div - intval
#     inliers = np.flatnonzero( np.logical_and( np.abs(intval) < 50, np.abs(rem) < tol ) )
#     return inliers
# 
# def gcd_ransac(vals, N=0, tol=0.01):
#     if N == 0:
#         N = len(vals)
#         
#     best_val = 0
#     inliers = []
#     for i in range(N):
#         ran_vals = np.random.choice(vals, 2, replace=False)
#         gcd = gcd_approx(ran_vals[0], ran_vals[1])
#         print ran_vals[0], ran_vals[1], ' => ', gcd
# #         print gcd
#         if gcd < tol:
#             continue
#         inliers_cur = gcd_get_inliers(vals, gcd, tol)
#         print len(inliers_cur), inliers_cur[0].shape
#         if len(inliers_cur) > len(inliers):
#             print 'update: ', best_val, ' => ', gcd
#             inliers = inliers_cur
#             best_val = gcd
#             
#     if len(inliers) > 0.25*len(vals) and len(inliers) < len(vals):
#         for i in range(2,11):
#             gcd = best_val / i
#             inliers_cur = gcd_get_inliers(vals, gcd, tol)
#             if len(inliers_cur) > len(inliers):
#                 print 'correction update: ', best_val, ' => ', gcd
#                 inliers = inliers_cur
#                 best_val = gcd
#     return best_val, inliers
    
    
        
        
    
        