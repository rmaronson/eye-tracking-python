#!/usr/bin/env python

import os
import numpy as np
import csv
import time
import calendar

import file_info
import robot_position


def load_metadata(filename):
    root_dir = os.path.dirname(filename)
    gaze_index = file_info.get_index(os.path.basename(filename))
    
    timestamps = np.load(os.path.join(root_dir, '..', 'gaze', gaze_index, 'world_timestamps.npy'))
    print 'Gaze index:', gaze_index
    
    info = {}
    with open(os.path.join(root_dir, '..', 'gaze', gaze_index, 'info.csv')) as f:
        reader = csv.DictReader(f)
        for row in reader:
            info[row['key']] = row['value']
    
    timestamps = timestamps - timestamps[0] + calendar.timegm(time.strptime(info['Start Date'] + info['Start Time'], '%d.%m.%Y%H:%M:%S'))+4*60*60 # 4 hour time zone offset
    
    offset_file = os.path.join(root_dir, '..', 'gaze', gaze_index, 'offset.txt')
    if os.path.exists(offset_file):
        timestamps = timestamps + np.loadtxt(offset_file)
    
    robot_pos = robot_position.RobotPosition(os.path.join(root_dir, '..', 'robot', 'trajdata_%s_export.csv' % gaze_index))
    
#     print 'Time: %.1f-%.1f' % (timestamps[0], timestamps[-1])
#     print 'Pos:  %.1f-%.1f' % (robot_pos.timestamps[0], robot_pos.timestamps[-1]) 
#     print info


    return timestamps, robot_pos
    
    
def load_stabilization(filename):
    root_dir = os.path.dirname(filename)
    trial_name, _ = os.path.splitext(os.path.basename(filename))
    stab_file = os.path.join(root_dir, '..', 'stabilize', '%s_stab.csv' % trial_name)
    ref_frame = [];
    tfs = [];
    if os.path.exists(stab_file):
        with open(stab_file, 'r') as stab:
            stab_reader = csv.DictReader(stab)
            for row in stab_reader:
                ref_frame.append(int(row['ref_frame']))
                tx = float(row['tx'])
                ty = float(row['ty'])
                th = float(row['theta'])
                s = float(row['s'])
                ct = np.cos(th)
                st = np.sin(th)
                tfs.append( np.array([ [ s*ct, -s*st, tx],
                                       [ s*st,  s*ct, ty] ]) )
    return ref_frame, tfs
