#!/usr/bin/env python


import sys
import yaml
import csv
import os

def export_statistics(root_dir):
    frame_id = 0
    filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id
    outfile = os.path.join(root_dir, 'all_extrinsics.csv')
    
    with open(outfile, 'wb') as out:
        writer = csv.writer(out, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['frame_id', 'x', 'y', 'z', 'r1', 'r2', 'r3'])
        while (os.path.exists(filename)):
            print 'Reading file: ', filename
            with open(filename, 'r') as infile:
                data = yaml.load(infile)
                if all(key in data for key in ['rvec', 'tvec']):
                    writer.writerow([frame_id] + [x for r in data['tvec' ] for x in r] + [x for r in data['rvec'] for x in r])
            
            frame_id = frame_id+1
            filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'usage: export_cal_info <dir_name(s)>'
        sys.exit(1)
        
    map(export_statistics, sys.argv[1:])
        
