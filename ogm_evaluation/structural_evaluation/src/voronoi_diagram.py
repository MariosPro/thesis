#!/usr/bin/env python

import cv2
import timeit
from scipy import ndimage
from scipy import weave
from geometry_msgs.msg import Point
from skimage.morphology import skeletonize
from skimage.morphology import medial_axis
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

# final pruning structuring elements

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

    def extractVoronoi(self, map, parameters):

        visualization = Visualization(map.frame_id)

        smoothedImage = map.mapImage
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

        kernel = np.ones((4, 4), np.uint8)
        if parameters['morphOpen']:
            # smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, 4)
            smoothedBinary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel,
                                              parameters['morphOpenIterations'])
        
        #  smoothedBinaryImage = np.zeros(smoothedBinary.shape, np.uint8)
        # smoothedBinaryImage[smoothedBinary > 0] = 255
        # # cv2.imshow("binaryImage", binaryImage)
        # cv2.imshow("smoothedMap", smoothedBinaryImage)
        # # cv2.imshow("Map", map.mapImage)
# #         cv2.imshow("smoothedImage", smoothedImage)
        # cv2.waitKey(1000)

        # perform skeletonization
        start_time = timeit.default_timer()
        skeleton = self.skeletonization(smoothedBinary,
                                        parameters['skeletonizationMethod'])
        elapsed = timeit.default_timer() - start_time
        print "Skeletonization via ", parameters['skeletonizationMethod'], \
            " execution time (ms): ", elapsed * 1000

        # skeletonImage = np.zeros(skeleton.shape)
        # skeletonImage[skeleton > 0] = 255
        # cv2.imshow("skeletonImage", skeletonImage)
        # cv2.waitKey(1000)
        # pruning of the skeleton
        if parameters['pruning']:
            start_time = timeit.default_timer()
            for i in range(0, parameters['pruningIterations']):
                skeleton = self.pruning(skeleton)
            elapsed = timeit.default_timer() - start_time
            print "Pruning execution time (ms): ", elapsed * 1000
            # prunedskeletonImage = np.zeros(skeleton.shape)
            # prunedskeletonImage[skeleton > 0] = 255
            # cv2.imshow("prunedskeletonImage", prunedskeletonImage)
            # cv2.waitKey(1000)

            # # final pruning of the skeleton
            start_time = timeit.default_timer()
            skeleton = self.finalPruning(skeleton)
            elapsed = timeit.default_timer() - start_time
            print "Final pruning execution time (ms): ", elapsed * 1000

        vizPoints = []
        for i in range(skeleton.shape[0]):
            for j in range(skeleton.shape[1]):
                if skeleton[i][j] == 1:
                    p = Point()
                    p.x = j
                    p.y = map.height - i
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

    def skeletonization(self, binary, method):
        if method == "zhangSuenThinning":
            skeleton = self.thinning(binary)
        if method == "medial_axis":
            skeleton = medial_axis(binary)
        if method == "thinning":
            skeleton = skeletonize(binary)
        return skeleton

    def thinningIteration(self, im, iter):
        I, M = im, np.zeros(im.shape, np.uint8)
        expr = """
        for (int i = 1; i < NI[0]-1; i++) {
            for (int j = 1; j < NI[1]-1; j++) {
                int p2 = I2(i-1, j);
                int p3 = I2(i-1, j+1);
                int p4 = I2(i, j+1);
                int p5 = I2(i+1, j+1);
                int p6 = I2(i+1, j);
                int p7 = I2(i+1, j-1);
                int p8 = I2(i, j-1);
                int p9 = I2(i-1, j-1);
                int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                         (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                         (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                         (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
                int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);
                if (A == 1 && B >= 2 && B <= 6 && m1 == 0 && m2 == 0) {
                    M2(i,j) = 1;
                }
            }
        } 
        """

        weave.inline(expr, ["I", "iter", "M"])
        return (I & ~M)


    def zhangSuenThinning(self, src):
        dst = src.copy() / 255
        prev = np.zeros(src.shape[:2], np.uint8)
        diff = None

        while True:
            dst = self.thinningIteration(dst, 0)
            dst = self.thinningIteration(dst, 1)
            diff = np.absolute(dst - prev)
            prev = dst.copy()
            if np.sum(diff) == 0:
                break

        return dst * 255

    def pruning(self, skeleton):
        prunedSkeleton = skeleton

        removedBranches = []
        for i in xrange(0, len(pruningHitKernel)):
            removedBranches.append(ndimage.binary_hit_or_miss(prunedSkeleton,
                                                              structure1 =
                                                              pruningHitKernel[i],
                                                              structure2 =
                                                              pruningMissKernel[i]).astype(int))
        finalRemovedBranches = removedBranches[0]

        for k in xrange(1, len(removedBranches)):
            finalRemovedBranches = np.logical_or(finalRemovedBranches,
                                                 removedBranches[k]).astype(int)
 
        prunedSkeleton = np.logical_and(skeleton,
                                        np.logical_not(finalRemovedBranches).
                                        astype(int)).astype(int)
        
        # start_time = timeit.default_timer()
        # prunedSkeleton = self.skeletonization(prunedSkeleton, 'morphology')
        # elapsed = timeit.default_timer() - start_time
        # print "pruning thinning iteration execution time (ms): ", elapsed * 1000

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

