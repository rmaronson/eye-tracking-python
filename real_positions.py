#!/usr/bin/env python

import numpy as np

def get_grid_points_3d(points2d):
    points3d_grid = np.concatenate([points2d, np.ones([points2d.shape[0], 1])], axis=1)
    offset = np.array([24*0.0254, 0, 0])
    scale = 0.0254
    shift_mat = np.array([ [-scale,     0, 0],
                           [     0, scale, 0],
                           [     0,     0,  1] ], dtype=float)
    return np.dot(shift_mat, points3d_grid.T).T + offset


