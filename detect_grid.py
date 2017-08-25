#!/usr/bin/env python

import cv2
import numpy as np
from shapely.geometry import LineString, LinearRing, Point, MultiPoint
from shapely.prepared import prep
import matplotlib.pyplot as plt
import itertools
import scipy.optimize as opt
from scipy.spatial import distance



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
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 100
        params.thresholdStep = 5
        
        params.filterByArea = True
        params.maxArea = 80
        
        params.filterByConvexity = True
        params.minConvexity = 0.8
        
        self.detector = cv2.SimpleBlobDetector_create(params)
        
    def get_grid(self, frame, animate=False):
        blobs = self.detector.detect(frame)
        results = {'blobs': [list(b.pt) for b in blobs]}
                
        im_keypoints_only = np.zeros(frame.shape, dtype=frame.dtype)
    
        for key in blobs:
            int_pt = tuple([int(x) for x in key.pt])
            im_keypoints_only = cv2.circle(im_keypoints_only, int_pt, 0, (255,255,255), -1)
    
        lines = cv2.HoughLines(cv2.cvtColor(im_keypoints_only, cv2.COLOR_BGR2GRAY), 1, np.pi/180, 3, 40)
        results['lines'] = lines.tolist() if lines is not None else None
        
        if lines is not None and len(lines) > 4:
#             lines = get_line_pairs(lines)
            H, horz_idx, vert_idx, lines1, lines2 = get_rectification_ransac(np.array(lines))#, im_keypoints_only)
            
            if H is not None:
                results['H'] = H.tolist()
                all_pts = np.array([[key.pt for key in blobs]])
                rect_pts = np.squeeze(cv2.perspectiveTransform(all_pts, H))
                
                inliers_guess = get_inlier_guess(rect_pts, horz_idx, vert_idx, lines1, lines2, radius_multiple=5)
                results['inliers_guess'] = inliers_guess
                if len(inliers_guess) > 3:
                    
    #                 inliers, grid_vals, grid_spacing = fit_grid(rect_pts, horz_idx, vert_idx, lines1, lines2)
                    inliers, grid_vals, grid_spacing = fit_grid_ransac(rect_pts[inliers_guess,:], N=500, tol=0.1)
    #                 print inliers, inliers_guess
                    inliers = [inliers_guess[il] for il in inliers]
                    results['inliers'] = inliers
                    results['grid_vals'] = grid_vals.tolist()
                    results['grid_spacing'] = grid_spacing.tolist()
                        
                    im_rect = np.zeros(frame.shape, dtype=frame.dtype)
                    im_orig = None
                    
                    if inliers is not None:
                        if animate:
                            for rect_idx in inliers:
                                p0 = tuple([int(r) for r in (rect_pts[rect_idx])])
                                px = tuple([int(r) for r in (rect_pts[rect_idx] + [grid_spacing[0], 0] )])
                                py = tuple([int(r) for r in (rect_pts[rect_idx] + [0, grid_spacing[1]] )])
                                im_rect = cv2.circle(im_rect, p0, 3, (255,255,255), -1)
                                im_rect = cv2.line(im_rect, p0, px, (0,255,0))
                                im_rect = cv2.line(im_rect, p0, py, (0,255,0))          
                            if grid_vals is not None:
                                im_orig = np.array(frame, copy=True)
                                im_orig = cv2.drawKeypoints(im_orig, blobs, im_orig, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                                im_orig = cv2.drawKeypoints(im_orig, [blobs[i] for i in inliers_guess], im_orig, (255,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #                         im_orig = cv2.drawKeypoints(im_orig, [blobs[i] for i in inliers], im_orig, (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                                for idx in range(len(inliers)):
                                    rect_idx = inliers[idx]
                                    p0 = tuple([int(r) for r in (rect_pts[rect_idx])])
                                    label = str(int(grid_vals[idx,0])) + ',' + str(int(grid_vals[idx,1]))
                                    im_rect = cv2.putText(im_rect, label, p0, cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255))
                                    
                                    p0_orig = tuple(int(r) for r in blobs[rect_idx].pt)
                                    im_orig = cv2.putText(im_orig, label, p0_orig, cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255))
                                    im_orig = cv2.circle(im_orig, p0_orig, 5, (0,0,255),1)          
                            cv2.imshow("Rectified dots", im_rect)
                            if im_orig is not None:
                                cv2.imshow("Grid points", im_orig)
                                                      
                        if grid_vals is not None:
                            points2d = np.array([blobs[il].pt for il in inliers])
                            results['points2d'] = points2d.tolist()
                            return points2d, grid_vals, results
        return None, None, results
                    

def get_inlier_guess(rect_pts, horz_idx, vert_idx, lines1, lines2, dist=300, radius_multiple=5):
    accepted_pts = set()
    for (horz, vert) in itertools.product(horz_idx, vert_idx):
        hp1, hp2 = lines1[horz], lines2[horz]
        vp1, vp2 = lines1[vert], lines2[vert]
        
        hu = hp2 - hp1
        vu = vp2 - vp1
        
        A = np.array([[ hu[0], -vu[0] ], [ hu[1], -vu[1] ]])
        b = np.array([ vp1[0] - hp1[0], vp1[1] - hp1[1] ])
        mult = np.linalg.solve(A, b)
        
        pt = np.mean( [hp1 + mult[0]*hu, vp1 + mult[1]*vu], 0)
        
        dists = np.sum( (rect_pts - pt)**2, 1 )
        idx = np.argmin(dists)
        if dists[idx] < 10:
            accepted_pts.add(idx)
    accepted_pts = list(accepted_pts)
    
    # Broaden the search to any nearby points
    hull = MultiPoint(rect_pts[accepted_pts,:]).convex_hull
    hull = prep(hull.buffer(np.sqrt(hull.area) * radius_multiple))
    inlier_guess = [i for i in range(rect_pts.shape[0]) if hull.contains(Point(rect_pts[i,:]))]
    
#     plt.scatter(rect_pts[:,0], rect_pts[:,1], c='r')
#     plt.scatter(rect_pts[inlier_guess,0], rect_pts[inlier_guess,1], c='y')
#     plt.scatter(rect_pts[accepted_pts,0], rect_pts[accepted_pts,1], c='g')
#     plt.show()
    
    return inlier_guess

def fit_grid(rect_pts, inlier_guess ):  
            
    if len(inlier_guess) > 2:
        accepted_pts = [pt for pt in inlier_guess]
        ds = np.array([ rect_pts[p1]-rect_pts[p2] for p1,p2 in itertools.combinations(accepted_pts, 2) ])
        print ds.shape
#                     dx, inl, mul = gcd_ransac(ds[:,0], tol=2, N=500)
#                     dy, inl, mul = gcd_ransac(ds[:,1], tol=2, N=500)
        dx, okx = gcd_optimize(ds[:,0], 5, 80)
        dy, oky = gcd_optimize(ds[:,1], 5, 80)
        print 'dx=', dx, ' (', okx, ') dy=', dy, ' (', oky, ')'
        
        if okx and oky:
            print rect_pts.shape
            ref_x = np.min(rect_pts[accepted_pts,0])
            ref_y = np.min(rect_pts[accepted_pts,1])
            x_coord = np.round((rect_pts[:,0] - ref_x) / dx)
            y_coord = np.round((rect_pts[:,1] - ref_y) / dy)
            return accepted_pts, np.array([x_coord, y_coord]).T, np.array([dx, dy])
        else:
            return accepted_pts, None, np.array([dx, dy]) 
    else:
        return None, None, None


def simplify_gcd(gcd, multiples):
    ch = False
#     print multiples
    mult_pos = abs(multiples) > 0.5
    if np.any(mult_pos):
        least_mult = np.min(multiples[mult_pos])
        if least_mult > 1:
            mmult = multiples / least_mult
            if np.all(np.round(mmult) < .5 / least_mult):
                gcd = gcd * least_mult
                multiples = mmult
                ch = True
    return gcd, multiples, ch

def fit_grid_ransac(pts, N=0, tol=.1, grid_size=20, animate=False):
    if N == 0:
        N = pts.shape[0]*10
    def get_grid_inliers(pts, grid_ref, grid_spacing, tol=.1, grid_size=20):
        grid_raw = np.array([p for p in itertools.product(range(-grid_size, grid_size+1), repeat=2)])
        grid = grid_ref + grid_raw*grid_spacing
        dists = distance.cdist(grid, pts, 'euclidean')

        # throw out repeated inliers
        min_dist_idxs, pts_idx = np.unique(np.argmin(dists, axis=0), return_index=True)
        
        within_tol_idx = dists[min_dist_idxs, pts_idx] < tol*np.linalg.norm(grid_spacing)
        return pts_idx[within_tol_idx], grid_raw[min_dist_idxs[within_tol_idx],:]
    
    inliers = []
    grid_vals = []
    grid_spacing = []
    grid_ref = []
    for _ in range(N):
        ran_idx = np.random.choice(range(pts.shape[0]), 2)
        cur_grid_ref = pts[ran_idx[0],:]
        cur_grid_spacing = np.abs(pts[ran_idx[1],:] - cur_grid_ref)
        cur_inliers, cur_grid_vals = get_grid_inliers(pts, cur_grid_ref, cur_grid_spacing, tol, grid_size)
        if len(cur_inliers) > len(inliers):
            inliers = cur_inliers
            grid_vals = cur_grid_vals
            grid_spacing = cur_grid_spacing
            grid_ref = cur_grid_ref
            
            if animate:
                grid_raw = np.array([p for p in itertools.product(range(-grid_size, grid_size+1), repeat=2)])
                grid = grid_ref + grid_raw*grid_spacing
                plt.scatter(grid[:,0], grid[:,1], c='k', marker='x')
                
                plt.scatter(pts[:,0], pts[:,1], c='r')
                plt.scatter(pts[inliers,0], pts[inliers,1], c='g')
                plt.show()
    
    scales = range(1,5) + [1./x for x in range(1,5)]
    for int_scale in itertools.product(scales, repeat=2): 
        cur_grid_spacing = grid_spacing * int_scale
        cur_inliers, cur_grid_vals = get_grid_inliers(pts, grid_ref, cur_grid_spacing, tol, grid_size)
#         print 'Checking scale ', cur_grid_spacing, ' (orig: ', grid_spacing, ')'
        if len(cur_inliers) > len(inliers):
            inliers = cur_inliers
            grid_vals = cur_grid_vals
            grid_spacing = cur_grid_spacing
            
            if animate:
                grid_raw = np.array([p for p in itertools.product(range(-grid_size, grid_size+1), repeat=2)])
                grid = grid_ref + grid_raw*grid_spacing
                plt.scatter(grid[:,0], grid[:,1], c='k', marker='x')
                
                plt.scatter(pts[:,0], pts[:,1], c='r')
                plt.scatter(pts[inliers,0], pts[inliers,1], c='g')
                plt.show()
        
    grid_spacing[0], grid_vals[:,0], _ = simplify_gcd(grid_spacing[0], grid_vals[:,0])
    grid_spacing[1], grid_vals[:,1], _ = simplify_gcd(grid_spacing[1], grid_vals[:,1])
    
    # Recenter the grid
    if len(inliers) > 0:
        xvals, xcounts = np.unique(grid_vals[:,0], return_counts=True)
        min_x_val = xvals[xcounts > 1][0] if np.any(xcounts > 1) else xvals[0]
        grid_vals[:,0] = grid_vals[:,0] - min_x_val
        
        yvals, ycounts = np.unique(grid_vals[:,1], return_counts=True)
        min_y_val = yvals[ycounts > 1][0] if np.any(ycounts > 1) else yvals[0]
        grid_vals[:,1] = grid_vals[:,1] - min_y_val
    
    return inliers, grid_vals, grid_spacing



def get_rectification_ransac(line_hough, frame=[], N=500, tol=0.02):
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
        return None, None, None, None, None
    
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
        
    for _ in range(N):
#         ran_lines = np.random.choice(range(len(line_pairs)), 4, replace=False)
        ran_g1 = np.random.choice(lines_g1, 2, replace=False)
        if line_hough[ran_g1[0],0,1] < line_hough[ran_g1[1],0,1]:
            ran_g1 = ran_g1[::-1]
            
        ran_g2 = np.random.choice(lines_g2, 2, replace=False)
        if line_hough[ran_g2[0],0,1] < line_hough[ran_g2[1],0,1]:
            ran_g2 = ran_g2[::-1]
            
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
#             print 'Update: ', H, ' horz=', len(horz_idx), ' vert=', len(vert_idx), ' uniq=', uniq
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
            
    if len(H) > 0:
        lines1 = np.squeeze(cv2.perspectiveTransform(lines_first, H))
        lines2 = np.squeeze(cv2.perspectiveTransform(lines_second, H))
        return H, horz_idx, vert_idx, lines1, lines2
    else:
        return None, None, None, None, None
            
            
        
        
        
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

def gcd_approx(a, b, tol=0.01):
    if b > a:
        a, b = b, a
    while b > tol:
        a, b = b, a%b
    return a
 
def gcd_get_inliers(vals, gcd, tol=0.01):
    div = vals / gcd
    intval = np.round(div)
    rem = div - intval
    inliers = np.flatnonzero( np.logical_and( np.abs(intval) < 50, np.abs(rem) < tol ) )
    return inliers
 
def gcd_ransac(vals, N=0, tol=0.01):
    if N == 0:
        N = vals.size**2
         
    best_val = 0
    inliers = []
    for i in range(N):
        ran_vals = np.random.choice(vals, 2, replace=False)
#     for ran_vals in itertools.combinations(vals, 2):
        gcd = abs(gcd_approx(ran_vals[0], ran_vals[1]))
#         print gcd
        if gcd < tol:
            continue
        inliers_cur = gcd_get_inliers(vals, gcd, tol)
        if not inliers_cur.size:
            continue
        if len(inliers_cur) > len(inliers):
#             print 'update: ', best_val, ' => ', gcd
            inliers = inliers_cur
            best_val = gcd
             
    if len(inliers) < len(vals):
        for i in range(2,11):
            gcd = best_val / i
            inliers_cur = gcd_get_inliers(vals, gcd, tol)
            if len(inliers_cur) > len(inliers):
#                 print 'correction update: ', best_val, ' => ', gcd
                inliers = inliers_cur
                best_val = gcd
                
    multiples = np.round(vals[inliers] / best_val)
#     least_mult = np.min(multiples)
#     if least_mult > 1:
#         mmult = multiples / least_mult
#         if np.all(np.round(mmult) < tol):
#             best_val = best_val * least_mult
#             multiples = mmult
    
    return best_val, inliers, multiples
    
    
def gcd_optimize(vals, guess_min=10, guess_max=30):
    def gcd_error(gcd, vals):
        vals = vals / gcd
        ints = np.round(vals)
        d = (vals - ints)/(ints+1)
        return np.sqrt(np.sum(np.abs(d)))
    
    vals = np.array(vals)
    res = opt.minimize_scalar(gcd_error, bounds=(guess_min, guess_max), args=(vals), method='bounded')
    print res
    
#     C = 20
#     t = np.array(range(4*C))*res.x/C + res.x/8
#     err = [gcd_error(v, all_vals) for v in t]
#     plt.plot(t/a,err)
#     plt.show()

    gcd = res.x
    ints = np.abs(np.round(vals / gcd))
    if np.any(ints > 0.5):
        minmult = np.min(ints[ints > 0.5])
        if minmult > 1:
            ints2 = ints / minmult
            if np.max(np.abs(ints2 - np.round(ints2))) < .5 / minmult:
                print 'Reduced ', gcd, ' => ', gcd * minmult
                gcd = gcd * minmult
    
    
    return gcd, res.success
        
        
    
        