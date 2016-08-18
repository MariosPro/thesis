#!/usr/bin/env python

import cv2
from scipy import ndimage
from geometry_msgs.msg import Point
from skimage.morphology import skeletonize
from visualization import Visualization
import numpy as np

# structuring elements for morphological operations

pruningHitKernel = []
pruningMissKernel = []
finalPruningKernel = []

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

#final pruning structuring elements

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


class VoronoiDiagram:
    
    def extractVoronoi(self, map):
        
        visualization = Visualization(map.frame_id)

        smoothedImage = map.mapImage
        smoothedImage = cv2.GaussianBlur(smoothedImage, (9, 9), 0)
        smoothedImage = cv2.medianBlur(smoothedImage, 11)
        binary = np.ones(map.mapImage.shape, np.uint8)
        binary[smoothedImage < 255] = 0

        binaryImage = np.zeros(map.ogm.shape)
        binaryImage[binary > 0] = 255

        kernel = np.ones((4, 4), np.uint8)
        # smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, 4)
        smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, 3)
        smoothedBinaryImage = np.zeros(smoothedBinary.shape)
        smoothedBinaryImage[smoothedBinary > 0] = 255
        # binary[ smoothedImage <= 50.0] = 1
        cv2.imshow("binaryImage", binaryImage)
        cv2.imshow("smoothedMap", smoothedBinaryImage)
        cv2.imshow("Map", map.mapImage)
        cv2.imshow("smoothedImage", smoothedImage)
        cv2.waitKey(1000)

        # perform skeletonization
        skeleton = skeletonize(smoothedBinary)

        # pruning of the skeleton
        skeleton = self.pruning(skeleton, 20)

        # final pruning of the skeleton
        skeleton = self.finalPruning(skeleton)

        voronoiPoints = []
        vizPoints = []
        for i in range(skeleton.shape[0]):
            for j in range(skeleton.shape[1]):
                if skeleton[i][j] == 1:
                    p = Point()
                    p.x = j
                    p.y = map.height - i
                    voronoiPoints.append(p)
                    vp = Point()
                    vp.x = j * map.resolution + map.origin['x']
                    vp.y = (map.height - i) * map.resolution + map.origin['y']
                    vizPoints.append(vp)

        print "Voronoi Diagram extracted"

        if map.frame_id == "visualization_map1":
            colors = [1.0, 1.0, 0.0, 0.0]
            ns = "map1/voronoiPoints"
        else:
            colors = [1.0, 0.0, 0.0, 1.0]
            ns = "map2/voronoiPoints"
        visualization.visualizePoints(map.frame_id,
                                    8,
                                    0,
                                    ns,
                                    0.02,
                                    colors,
                                    vizPoints,
                                    True)

        return skeleton

    def pruning(self, skeleton, iterations):
        removedBranches = []
        prunedSkeleton = skeleton
        for i in xrange(0, iterations):
            for i in xrange(0, len(pruningHitKernel)):
                removedBranches.append(ndimage.binary_hit_or_miss(prunedSkeleton,
                                                                  structure1 =
                                                                  pruningHitKernel[i],
                                                                  structure2 =
                                                                  pruningMissKernel[i]).astype(int))
            finalRemovedBranches = removedBranches[0]
            for i in xrange(1, len(removedBranches)):
                finalRemovedBranches = np.logical_or(finalRemovedBranches,
                                                     removedBranches[i]).astype(int)

            prunedSkeleton = np.logical_and(skeleton,
                                           np.logical_not(finalRemovedBranches).astype(int)).astype(int)

        return prunedSkeleton
    
    def finalPruning(self, skeleton):
        removedBranches = []
        prunedSkeleton = skeleton
        for i in xrange(0, len(finalPruningKernel)):
            removedBranches.append(ndimage.binary_hit_or_miss(prunedSkeleton,
                                                              finalPruningKernel[i]).astype(int))
        finalRemovedBranches = removedBranches[0]
        for i in xrange(1, len(removedBranches)):
            finalRemovedBranches = np.logical_or(finalRemovedBranches,
                                                 removedBranches[i]).astype(int)

        prunedSkeleton = np.logical_and(skeleton,
                                       np.logical_not(finalRemovedBranches).astype(int)).astype(int)

        return prunedSkeleton

