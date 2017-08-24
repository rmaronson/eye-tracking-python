#!/usr/bin/env python

import detect_grid
import numpy as np
import itertools
import sys

if __name__=="__main__":
    if len(sys.argv) > 1:
        np.random.seed(int(sys.argv[1]))
    
    grid_size = 10
    N_grid = .3*(grid_size**2)
    s = 0.05
    N_noise = 5
    
    grid_ref = np.random.randn(2)*10
    grid_spacing = np.random.rand(2)*10
    
#     grid_all = np.array([p for p in itertools.product(range(-grid_size, grid_size+1), repeat=2)])
#     grid_sel = np.random.choice(range(grid_all.shape[0]), int(N_grid))
#     grid_pts = grid_ref + grid_spacing*grid_all[grid_sel,:]
    grid_sel = [ tuple(np.random.randint(-grid_size, grid_size+1, 2)) ]
    steps = np.array([[1,0], [0,1], [-1,0], [0,-1]])
    while len(grid_sel) < N_grid:
        step_idx = np.random.choice(range(steps.shape[0]), 1)
        elem_idx = np.random.choice(range(len(grid_sel)), 1)
        grid_sel.append( tuple(grid_sel[elem_idx] + steps[step_idx,:].ravel()) )
        grid_sel = list(set(grid_sel))
#     print grid_sel
    grid_pts = grid_ref + grid_spacing*np.array(grid_sel)
    grid_pts = grid_pts + np.random.randn(*grid_pts.shape)*grid_spacing*s
    
    noise_pts = np.random.randn(N_noise, 2)*grid_size*np.linalg.norm(grid_spacing)*.5
    
    pts = np.concatenate([grid_pts, noise_pts])
    print pts
    
    calc_inliers, calc_grid_vals, calc_spacing = detect_grid.fit_grid_ransac(pts, N=grid_size*10, animate=True, grid_size=30)
    
    print 'Spacing: ', grid_spacing
    print 'Found spacing: ', calc_spacing
        