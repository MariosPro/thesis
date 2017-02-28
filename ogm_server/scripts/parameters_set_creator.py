#!/usr/bin/env python

import rospkg
import sys
import os
import yaml
import json
import numpy as np
from collections import OrderedDict
if __name__ == "__main__":
    detectors = ['SIFT', 'SURF', 'FAST', 'STAR', 'ORB', 'GFTT',
            'HARRIS']
    # descriptors = ['SIFT', 'SURF', 'ORB'] 
    descriptors = ['SIFT', 'SURF', 'ORB','ANNULAR STATISTICS','CIRCLE INTERSECTIONS', 'MEAN RAYS', 'ANNULAR+RAYS', 'ALL CUSTOMS']
    matchers = ['BruteForce', 'BruteForce-Hamming']
    matching_methods = ['SIMPLE']
    matching_ratio = np.arange(0.6, 0.8, 0.05)
    ransac_error = [-1] #np.arange(-1, 5, 1)
    dist_norm = ['Manhattan']
#     gaussianBlur1 = [0]
    # medianBlur1 = [0]
    gaussianBlur2 = [0, 1]
    medianBlur2 = [0, 1]
#     gK1 = [3, 5, 7, 9, 11, 13, 15]
    # mK1 = [3, 5, 7, 9, 11, 13, 15]
    gK2 = [3, 5, 7, 9, 11, 13, 15]#, 15, 17, 19]
    mK2 =  [3, 5, 7, 9, 11, 13, 15]#, 15, 17, 19]
    morphologicalFiltering = [0] #, 1]
    scale = [0,1]

    mr = 5
    count = 1
    params_keys = ['#','method', 'detector', 'descriptor', 'matcher',
        'matchingMethod', 'matchingRatio', 'ransacReprjError', 'distNorm',
        'gaussianBlur1', 'medianBlur1', 'gaussianBlur2', 'medianBlur2',
        'gaussianKernel1', 'medianKernel1', 'gaussianKernel2',
        'medianKernel2','morphologicalFiltering',
        'x', 'y','theta','scale','slamOffsetScale', 'groundTruthOffsetscale', 'scaleMapsBrushfire', 'binary', 'manualAlignment']
    parameters_yaml_path =rospkg.RosPack().get_path("ogm_server")+'/scripts/scale_parameters.json'
    combs=list()
    for det in detectors:
        for dec in descriptors:
            if det == "SIFT"  and dec == "ORB":
                continue
            for m in matchers:
                if dec == 'ORB' and (m!= 'BruteForce-Hamming'):
                    continue
                if dec != 'ORB' and (m == 'BruteForce-Hamming'):
                    continue
                for mm in matching_methods:
                    # for mr in matching_ratio:
                    for r in ransac_error:
                        for dn in dist_norm:
#                             for gb1 in gaussianBlur1:
                                # for mb1 in medianBlur1:
                                    for gb2 in gaussianBlur2:
                                         for mb2 in medianBlur2:
 #                                                if gb1 == 0:
                                                    # gaussianKernel1 = [0]
                                                # else:
                                                    # gaussianKernel1 = gK1
#                                                    for g1 in gaussianKernel1:
                                                    # if mb1 == 0:
                                                        # medianKernel1 = [0]
                                                    # else:
                                                        # medianKernel1 = mK1
                                                    # for m1 in medianKernel1:
                                                if gb2 == 0:
                                                    gaussianKernel2 = [0]
                                                else:
                                                    gaussianKernel2 = gK2
                                                for g2 in gaussianKernel2:
                                                    if mb2 == 0:
                                                        medianKernel2 = [0]
                                                    else:
                                                        medianKernel2 = mK2
                                                    for m2 in medianKernel2:
                                                        for mf in morphologicalFiltering:
                                                            for s in scale:
                                                                params_values=[count,'FEATURES',
                                                                            det,
                                                                            dec,
                                                                            m,
                                                                            mm,
                                                                            str(mr),
                                                                            str(r),
                                                                            dn,
                                                                            gb2,
                                                                            mb2,
                                                                            gb2,
                                                                            mb2,
                                                                            str(g2),
                                                                            str(m2),
                                                                            str(g2),
                                                                            str(m2), 
                                                                            mf,
                                                                            0, 0, 0, 1, 1, 1, s, 0, 0]
                                                                params=OrderedDict(zip(params_keys,
                                                                    params_values))
                                                              #   print\
                                                                # params
                                                                combs.append(params)
                                                                count = count + 1
    with open(parameters_yaml_path, 'w') as f:
        json.dump([c for c in combs],f,sort_keys=False, indent=4 )
    print "Produced %d feature matching parameters combinations" % (count-1)
