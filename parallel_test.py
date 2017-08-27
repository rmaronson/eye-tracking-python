#!/usr/bin/env python

import multiprocessing
import itertools

def f(a):
    print a
    return a[0]**a[1]

def g(y):
    def h(x):
        return f(x,y)
    return h
    

if __name__=="__main__":
    pool = multiprocessing.Pool()
    
    for i in pool.imap_unordered(f, itertools.izip_longest(range(10), [], fillvalue=2)  ):
        print i
