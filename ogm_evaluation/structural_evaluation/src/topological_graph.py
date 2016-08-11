#!/usr/bin/env python

import cv2
import numpy as np
from scipy import ndimage
from voronoi_diagram import VoronoiDiagram
from visualization import Visualization
from geometry_msgs.msg import Point

# structuring elements for morphological operations

edgesKernel = []
crossesHitKernel = []
crossesMissKernel = []
edgesKernel.append(np.array((
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesKernel.append(np.array((
    [0, 1, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesKernel.append(np.array((
    [0, 0, 1],
    [0, 1, 0],
    [0, 0, 0]), np.uint8))

edgesKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 1],
    [0, 0, 0]), np.uint8))

edgesKernel.append(np.array((
    [0, 0, 0],
    [1, 1, 0],
    [0, 0, 0]), np.uint8))

edgesKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [1, 0, 0]), np.uint8))

edgesKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 1, 0]), np.uint8))

edgesKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 1]), np.uint8))

crossesHitKernel.append(np.array((
    [1, 0, 0],
    [1, 1, 0],
    [0, 0, 0])))

crossesMissKernel.append(np.array((
    [0, 1, 0],
    [0, 0, 0],
    [1, 1, 0])))

crossesHitKernel.append(np.array((
    [0, 0, 0],
    [1, 1, 0],
    [1, 0, 0])))

crossesMissKernel.append(np.array((
    [1, 1, 0],
    [0, 0, 0],
    [0, 1, 0])))

crossesHitKernel.append(np.array((
    [0, 1, 1],
    [0, 1, 0],
    [0, 0, 0])))

crossesMissKernel.append(np.array((
    [1, 0, 0],
    [1, 0, 1],
    [0, 0, 0])))

crossesHitKernel.append(np.array((
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 0])))

crossesMissKernel.append(np.array((
    [0, 0, 1],
    [1, 0, 1],
    [0, 0, 0])))

crossesHitKernel.append(np.array((
    [0, 0, 1],
    [0, 1, 1],
    [0, 0, 0])))

crossesMissKernel.append(np.array((
    [1, 1, 0],
    [1, 0, 0],
    [0, 0, 0])))

crossesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 1],
    [0, 0, 1])))

crossesMissKernel.append(np.array((
    [0, 1, 1],
    [0, 0, 0],
    [0, 1, 0])))

crossesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [1, 1, 0])))

crossesMissKernel.append(np.array((
    [1, 1, 0],
    [1, 0, 0],
    [0, 0, 0])))

crossesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 1, 1])))

crossesMissKernel.append(np.array((
    [0, 1, 1],
    [0, 0, 1],
    [0, 0, 0])))


#Class for extracting Topological Graph from OGM
class TopologicalGraph:
    
    #Constructor
    def __init__(self):
        self.voronoiDiagram = VoronoiDiagram()

    def extractTopologicalGraph(self, map):
        visualization = Visualization(map.frame_id)
        print "TopologicalGraph instance Created %s" % (map.frame_id)
        self.voronoi = self.voronoiDiagram.extractVoronoi(map)
        voronoiVertices = self.findVertices(self.voronoi)

       #visualize in rviz
        vizPoints = []
        for i in xrange(voronoiVertices.shape[0]):
            for j in xrange(voronoiVertices.shape[1]):
                if voronoiVertices[i][j] == 1:
                    p = Point()
                    p.x = j *map.resolution + map.origin['x']
                    p.y = (map.height -i) * map.resolution + map.origin['y']
                    vizPoints.append(p)
        if map.frame_id == "visualization_map1":
            ns = "map1/voronoiVertices"
        else:
            ns = "map2/voronoiVertices"
        colors = [1.0, 0.0, 1.0, 0.0]
        visualization.visualize(map.frame_id,
                                8,
                                0,
                                ns,
                                0.02,
                                colors,
                                vizPoints)

    def findVertices(self, voronoi):

        voronoi = voronoi.astype(int)
        voronoiEdges = []
        voronoiCrosses = []
        voronoiImage = np.zeros((voronoi.shape), np.uint8)
        voronoiImage[voronoi > 0] = 255

        # extract voronoi edges from every diffent structuring element
        for kernel in edgesKernel :
            voronoiEdges.append(ndimage.binary_hit_or_miss(voronoi,
                kernel).astype(int))

        finalVoronoiEdges = voronoiEdges[0]
        for i in xrange(1, len(voronoiEdges)):
            finalVoronoiEdges = np.logical_or(finalVoronoiEdges,
                    voronoiEdges[i]).astype(int)

        finalVoronoiEdgesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        finalVoronoiEdgesImage[finalVoronoiEdges > 0] = 255
        
        # extract voronoi crosses from every diffent structuring element
        for i in xrange(0, len(crossesHitKernel)):
            voronoiCrosses.append(ndimage.binary_hit_or_miss(voronoi,
                structure1=crossesHitKernel[i], structure2=crossesMissKernel[i]).astype(int))

        finalVoronoiCrosses = voronoiCrosses[0]
        for i in xrange(1, len(voronoiCrosses)):
            finalVoronoiCrosses = np.logical_or(finalVoronoiCrosses,
                    voronoiCrosses[i]).astype(int)

        finalVoronoiCrossesImage = np.zeros((finalVoronoiCrosses.shape), np.uint8)
        finalVoronoiCrossesImage[finalVoronoiCrosses > 0] = 255

        # create the final voronoi vertices array
        voronoiVertices = np.logical_or(finalVoronoiEdges,
                finalVoronoiCrosses).astype(int)

        voronoiVerticesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        voronoiVerticesImage[voronoiVertices > 0] = 255

        return voronoiVertices

 
        cv2.imshow('voronoi', voronoiImage)
        cv2.imshow('voronoiEdges', finalVoronoiEdgesImage)
        cv2.imshow('voronoiCrosses', finalVoronoiCrossesImage)
        cv2.imshow("voronoiVertices", voronoiVerticesImage)
        cv2.waitKey(1000)

