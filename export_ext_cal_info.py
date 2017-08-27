#!/usr/bin/env python


import sys
import yaml
import csv
import os
# import numpy as np
# import scipy.sparse.csgraph as csg
# from scipy.spatial import distance

def export_statistics(root_dir):
    frame_id = 0
    filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id
    outfile = os.path.join(root_dir, 'all_extrinsics.csv')
#     outfile_filt = os.path.join(root_dir, 'all_extrinsics_filtered.csv')
#     ts = []
#     rs = []
#     frame_ids = []
    
    with open(outfile, 'wb') as out:
        writer = csv.writer(out, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['frame_id', 'x', 'y', 'z', 'r1', 'r2', 'r3'])
        while (os.path.exists(filename)):
            print 'Reading file: ', filename
            with open(filename, 'r') as infile:
                data = yaml.load(infile)
                if all(key in data for key in ['rvec', 'tvec']):
                    writer.writerow([frame_id] + [x for r in data['tvec' ] for x in r] + [x for r in data['rvec'] for x in r])
#                     frame_ids.append(frame_id)
#                     ts.append([x for r in data['tvec' ] for x in r])
#                     rs.append([x for r in data['rvec'] for x in r])
            
            frame_id = frame_id+1
            filename = os.path.join(root_dir, 'ext_cal_%05d.yaml') % frame_id
#             
#     if len(frame_ids) > 0:
#         ts = np.array(ts)
#         rs = np.array(rs)
#         dists = distance.cdist(ts, ts, 'euclidean')
#         n, bins = csg.connected_components(dists < 0.1, directed=False)
#         ok = bins == np.argmax(np.sum(i == bins) for i in range(n))
#         data = np.array((frame_ids, ts[:,0], ts[:,1], ts[:,2], rs[:,0], rs[:,1], rs[:,2], ok))
#         np.savetxt(outfile_filt, data.T, fmt=['%d'] + ['%10.5f']*6 + ['%d'], delimiter=',', 
#                    header='frame_id,x,y,z,r1,r2,r3,ok')
        

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'usage: export_cal_info <dir_name(s)>'
        sys.exit(1)
        
    map(export_statistics, sys.argv[1:])
        
