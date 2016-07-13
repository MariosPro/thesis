#!/usr/bin/env python

import rospkg
import sys
import os
import yaml
import numpy as np

if __name__ == "__main__":
    detectors = ['SIFT', 'SURF', 'FAST', 'STAR', 'ORB', 'GFTT',
            'HARRIS']
    descriptors = ['SIFT', 'SURF', 'ORB', 'RADIUS STATISTICS',
            'CIRCLE INTERSECTIONS', 'MEAN RAYS', 'ALL CUSTOMS']
    matchers = ['BruteForce', 'BruteForce-L1', 'BruteForce-Hamming']
            
    matching_methods = ['SIMPLE', 'RATIO', 'CROSSCHECK']
    matching_ratio = np.arange(0.5, 1.00, 0.05)
    ransac_error = np.arange(-1, 8, 1)
    dist_norm = ['Manhattan']
    count = 1
    parameters_yaml_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/parameters.yaml'
    with open(parameters_yaml_path, 'w') as f:
        for det in detectors:
            for dec in descriptors:
                for m in matchers:
                    if dec == 'ORB' and (m!= 'BruteForce-Hamming' or
                            m!= 'BruteForce-Hamming(2)'):
                        continue;
                    if dec != 'ORB' and (m == 'BruteForce-Hamming' or m ==
                            'BruteForce-Hamming(2)'):
                        continue;
                    for mm in matching_methods:
                        for mr in matching_ratio:
                            for r in ransac_error:
                                for dn in dist_norm:
                                    data = dict()
                                    data[count] = ['FEATURES', det, dec, m, mm, str(mr), str(r),
                                            dn, 0, 0, 0, 1, 1, 1, 0, 0, 0]
                                    # print data
                                    count = count + 1
                                    f.write(yaml.dump(data, default_flow_style=False))
    print "Produced %d feature matching parameters combinations" % count
