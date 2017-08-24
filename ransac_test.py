#!/usr/bin/env python


import numpy as np
import sys
import matplotlib.pyplot as plt
import detect_grid
import itertools
import scipy.optimize as opt

# def noisy_gcd(xs):
#     fs = 0.1
#     N = 2**10
#     t = np.array(range(N))*fs
#     v = np.zeros(t.shape)
#     for x in xs:
#         v = v + np.cos(2*np.pi*x*t/fs)
#     
#     v = v - np.mean(v)
#     plt.plot(t,v)
#     plt.show()
#     r = np.absolute(np.fft.rfft(v))
#     plt.plot(range(r.size), r)
#     plt.show()
#     return np.argmax(r)/float(N)



    
if __name__=="__main__":
    if len(sys.argv) > 1:
        np.random.seed(int(sys.argv[1]))
    a = np.random.rand() * 10
    N = 7
    Nn = 0
    s = 0.05
    M = 6
    ints = np.random.randint(0, M, N)
    vals = ints*a + np.random.randn(N)*a*s
    print 'a=', a
    print 'ints', ints, ' vals=', vals
    diffs = [abs(x-y) for x,y in itertools.combinations(vals, 2)]
    print 'diffs=', diffs
    
    noise = np.random.randn(Nn)*a*5
    
    all_vals = np.concatenate([diffs, noise])
      
#     C = 10
#     t = np.array(range(4*C))*a/C + a/8
#     err = [gcd_error(v, all_vals) for v in t]
#     plt.plot(t/a,err)
#     plt.show()
    
    gcd, inliers, multiples = detect_grid.gcd_ransac(all_vals, tol=0.01)
    gcd2 = gcd_optimize(all_vals, 0.1,10)
    print 'all_vals=', all_vals
    print 'a=', a
#     print 'ints=', ints
#
    print 'gcd=', gcd, ' gcd_opt=', gcd2
#     print 'inliers=', all_vals[inliers]
# #     print 'ints=', ints[inliers[inliers < ints.size]]
#     print 'mult=', multiples 
