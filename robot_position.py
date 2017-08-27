#!/usr/bin/env python

import numpy as np
import csv
import itertools
import scipy.interpolate


class RobotPosition:
    keys = ['mico_link_' + a + '_' + b for a,b in itertools.product(['1', '2', '3', '4', '5', 'hand'], ['x','y','z'])]
    def __init__(self, filename):
        with open(filename) as f:
            timestamps = []
            data = []
            reader = csv.DictReader(f)
            for row in reader:
                timestamps.append(float(row['time']))
                data.append([ float(row[k]) for k in RobotPosition.keys])
        self.timestamps = np.array(timestamps)
        self.data = np.array(data).reshape([len(timestamps), -1, 3])
        self.interpolater = scipy.interpolate.interp1d(self.timestamps, self.data, axis=0, copy=False, assume_sorted=True)
        
    def get_data(self, timestamp):
        if timestamp < self.timestamps[0]:
            return self.data[0,:,:]
        elif timestamp > self.timestamps[-1]:
            return self.data[-1,:,:]
        else:
            return self.interpolater(timestamp)
