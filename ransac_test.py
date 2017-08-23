#!/usr/bin/env python


import numpy as np
import detect_grid
import sys

if __name__=="__main__":
    if len(sys.argv) > 1:
        np.random.seed(int(sys.argv[1]))
    a = np.random.rand() * 10
    N = 20
    Nn = 0
    s = 0.0001
    ints = np.random.randint(-10, 10, N)
    vals = ints*a + np.random.randn(N)*a*s
    noise = np.random.randn(Nn)*a*5
    print a, ints, vals, noise
    all_vals = np.concatenate([vals, noise])
    
    gcd, inliers = detect_grid.gcd_ransac(all_vals)
    
    print 'a=', a, '\n', 'ints=', ints
    print 'all_vals=', all_vals
    
    print 'gcd=', gcd, 'inliers=', inliers
