#!/usr/bin/env python

import cv2
import timeit
from scipy import ndimage
from scipy import weave
from geometry_msgs.msg import Point
from skimage.morphology import skeletonize
from skimage.morphology import medial_axis
import thinning
import mahotas as mh
# from _cpp_functions import ffi, lib
from map import Map
from visualization import Visualization
import numpy as np
import scipy.misc
class VoronoiDiagram:

    def extractVoronoi(self, map, parameters):

        visualization = Visualization(map.frame_id)
        smoothedImage = map.mapImage
        scipy.misc.imsave('/home/marios/Desktop/original.png', smoothedImage)

        if map.frame_id == "visualization_map1" \
                and parameters['gaussianBlur1']:
            kernel = parameters['gaussianKernel1']
            smoothedImage = cv2.GaussianBlur(smoothedImage,
                                             (kernel, kernel), 0)
        if map.frame_id == "visualization_map2" \
                and parameters['gaussianBlur2']:
            kernel = parameters['gaussianKernel2']
            smoothedImage = cv2.GaussianBlur(smoothedImage, 
                                             (kernel, kernel), 0)
        if map.frame_id == "visualization_map1" \
                 and parameters['medianBlur1']:
                kernel = parameters['medianKernel1']
                smoothedImage = cv2.medianBlur(smoothedImage,
                                               kernel)
        if map.frame_id == "visualization_map2" \
                and parameters['medianBlur2']:
            kernel = parameters['medianKernel2']
            smoothedImage = cv2.medianBlur(smoothedImage, 
                                           kernel)
      
        binary = np.ones(map.mapImage.shape, np.uint8)
        binary[smoothedImage < 255] = 0
        smoothedBinary = binary
 
        # binaryImage = np.zeros(map.ogm.shape)
        # binaryImage[binary > 0] = 255

        kernel = np.ones((8, 8), np.uint8)
        if parameters['morphOpen']:
            # smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, 4)
            smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel,
                                              parameters['morphOpenIterations'])
        
        smoothedBinaryImage = np.zeros(smoothedBinary.shape, np.uint8)
        smoothedBinaryImage[smoothedBinary > 0] = 255
        scipy.misc.imsave('/home/marios/Desktop/test3.png', smoothedBinaryImage)
        
        #find useful map limits
        start_time = timeit.default_timer()
        map.findUsefulBoundaries(smoothedBinary)
        smoothedBinary = smoothedBinary[map.limits['min_x']:map.limits['max_x'],
                                        map.limits['min_y']:map.limits['max_y']]
        smoothedBinary = np.asarray(smoothedBinary, order='C')
        elapsed = timeit.default_timer() - start_time
        print "Find useful map limits execution time (ms): ", elapsed * 1000

        # perform skeletonization
        start_time = timeit.default_timer()
        skeleton = self.skeletonization(smoothedBinary,
                                        parameters['skeletonizationMethod'])
        elapsed = timeit.default_timer() - start_time
        print "Skeletonization via ", parameters['skeletonizationMethod'], \
            " execution time (ms): ", elapsed * 1000
   
        # pruning of the skeleton
        if parameters['pruning']:
            start_time = timeit.default_timer()
          #   for i in xrange(0, parameters['pruningIterations']):
                # skeleton = self.pruning(skeleton)
            skeleton = self.pruning2(skeleton, parameters['pruningIterations'])
            elapsed = timeit.default_timer() - start_time
            print "Pruning execution time (ms): ", elapsed * 1000
            
            # # final pruning of the skeleton
            start_time = timeit.default_timer()
            skeleton = self.finalPruning(skeleton)
            elapsed = timeit.default_timer() - start_time
            print "Final pruning execution time (ms): ", elapsed * 1000

         
        voronoiPoints = np.argwhere(skeleton == 1)
        voronoiPoints += [map.limits['min_x'], map.limits['min_y']]
     #    print vizPoints
        # for i in range(skeleton.shape[0]):
            # for j in range(skeleton.shape[1]):
                # if skeleton[i][j] == 1:
                    #  vp = Point()
                    # vp.x = j * map.resolution + map.origin['x']
                    # vp.y = (map.height - i) * map.resolution + map.origin['y']
                    # vizPoints.append(vp)

        # print "Voronoi Diagram extracted"

        # if map.frame_id == "visualization_map1":
            # colors = [1.0, 1.0, 0.0, 0.0]
            # ns = "map1/voronoiPoints"
        # else:
            # colors = [1.0, 0.0, 0.0, 1.0]
            # ns = "map2/voronoiPoints"
        # visualization.visualizePoints(map.frame_id,
                                      # 8,
                                      # 0,
                                      # ns,
                                      # 0.02,
                                      # colors,
                                      # vizPoints,
                                      # True)

        return skeleton, voronoiPoints.tolist()

    def skeletonization(self, binary, method):
        if method == "guoHallThinning":
            skeleton = thinning.guo_hall_thinning(binary)
        if method == "medial_axis":
            skeleton = medial_axis(binary)
        if method == "thinning":
            skeleton = skeletonize(binary)
        return skeleton
    
    def endPoints(self, skel):
        endpoint1=np.array([[0, 0, 0],[0, 1, 0],[2, 1, 2]])
        endpoint2=np.array([[0, 0, 0],[0, 1, 2],[0, 2, 1]])
        endpoint3=np.array([[0, 0, 2],[0, 1, 1],[0, 0, 2]])
        endpoint4=np.array([[0, 2, 1],[0, 1, 2],[0, 0, 0]])
        endpoint5=np.array([[2, 1, 2],[0, 1, 0],[0, 0, 0]])
        endpoint6=np.array([[1, 2, 0],[2, 1, 0],[0, 0, 0]])
        endpoint7=np.array([[2, 0, 0],[1, 1, 0],[2, 0, 0]])
        endpoint8=np.array([[0, 0, 0],[2, 1, 0],[1, 2, 0]])
        ep1=mh.morph.hitmiss(skel,endpoint1)
        ep2=mh.morph.hitmiss(skel,endpoint2)
        ep3=mh.morph.hitmiss(skel,endpoint3)
        ep4=mh.morph.hitmiss(skel,endpoint4)
        ep5=mh.morph.hitmiss(skel,endpoint5)
        ep6=mh.morph.hitmiss(skel,endpoint6)
        ep7=mh.morph.hitmiss(skel,endpoint7)
        ep8=mh.morph.hitmiss(skel,endpoint8)
        ep = ep1+ep2+ep3+ep4+ep5+ep6+ep7+ep8
        return ep

    def pruning2(self, skeleton, size):

        for i in xrange(0, size):
            endpoints = self.endPoints(skeleton)
            endpoints = np.logical_not(endpoints)
            skeleton = np.logical_and(skeleton,endpoints)
        return skeleton
  
    def pruning(self, skeleton):
        pruningHitKernel = []
        pruningMissKernel = []

        # pruning structuring elements
        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 1, 0],
            [1, 0, 0],
            [1, 1, 1])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 1, 1],
            [1, 0, 0],
            [1, 1, 0])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 0, 0],
            [1, 0, 1],
            [1, 1, 1])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 1, 1],
            [1, 0, 1],
            [1, 0, 0])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 1, 1],
            [1, 0, 1],
            [0, 0, 1])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [1, 1, 1],
            [0, 0, 1],
            [0, 1, 1])))

        pruningHitKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0])))

        pruningMissKernel.append(np.array((
            [0, 1, 1],
            [0, 0, 1],
            [1, 1, 1])))

        prunedSkeleton = skeleton

        removedBranches = []
        removedBranches = [ndimage.binary_hit_or_miss(prunedSkeleton,
                                                      structure1 =
                                                      pruningHitKernel[i],
                                                      structure2 =
                                                      pruningMissKernel[i]).astype(int)
                           for i in xrange(0, len(pruningHitKernel))]
       
        finalRemovedBranches = reduce(lambda x, y: x+y, removedBranches)
       
        prunedSkeleton = np.logical_and(skeleton,
                                    np.logical_not(finalRemovedBranches).
                                    astype(int)).astype(int)

        return prunedSkeleton

    def finalPruning(self, skeleton):

        # final pruning structuring elements

        finalPruningKernel = []

        finalPruningKernel.append(np.array((
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 0])))

        finalPruningKernel.append(np.array((
            [0, 0, 1],
            [0, 1, 1],
            [0, 0, 0])))

        finalPruningKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [0, 1, 1])))

        finalPruningKernel.append(np.array((
            [0, 0, 0],
            [1, 1, 0],
            [1, 0, 0])))

        finalPruningKernel.append(np.array((
            [0, 1, 1],
            [0, 1, 0],
            [0, 0, 0])))

        finalPruningKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 1],
            [0, 0, 1])))

        finalPruningKernel.append(np.array((
            [0, 0, 0],
            [0, 1, 0],
            [1, 1, 0])))

        finalPruningKernel.append(np.array((
            [1, 0, 0],
            [1, 1, 0],
            [0, 0, 0])))

        removedBranches = []
        prunedSkeleton = skeleton
        removedBranches = [ndimage.binary_hit_or_miss(prunedSkeleton,
                                                      finalPruningKernel[i]).astype(int)
                           for i in xrange(0, len(finalPruningKernel))]
        
        finalRemovedBranches = reduce(lambda x, y: x+y, removedBranches)
 
        prunedSkeleton = np.logical_and(skeleton,
                                        np.logical_not(finalRemovedBranches).astype(int)).astype(int)

        return prunedSkeleton

