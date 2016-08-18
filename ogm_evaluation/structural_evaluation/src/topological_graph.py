#!/usr/bin/env python

import cv2
import numpy as np
from scipy import ndimage
from voronoi_diagram import VoronoiDiagram
from visualization import Visualization
from geometry_msgs.msg import Point

# structuring elements for morphological operations

edgesHitKernel = []
edgesMissKernel = []
junctionsHitKernel = []
junctionsMissKernel = []

edgesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesMissKernel.append(np.array((
    [0, 0, 0],
    [1, 0, 1],
    [1, 1, 1])))

edgesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesMissKernel.append(np.array((
    [1, 1, 0],
    [1, 0, 0],
    [1, 1, 0])))

edgesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesMissKernel.append(np.array((
    [1, 1, 1],
    [1, 0, 1],
    [0, 0, 0])))

edgesHitKernel.append(np.array((
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 0])))

edgesMissKernel.append(np.array((
    [0, 1, 1],
    [0, 0, 1],
    [0, 1, 1])))

junctionsHitKernel.append(np.array((
    [0, 0, 1],
    [1, 1, 0],
    [0, 1, 0])))

junctionsMissKernel.append(np.array((
    [0, 1, 0],
    [0, 0, 1],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 1],
    [0, 1, 0])))

junctionsHitKernel.append(np.array((
    [0, 1, 0],
    [0, 1, 1],
    [1, 0, 0])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 0],
    [0, 1, 1],
    [0, 1, 0])))

junctionsMissKernel.append(np.array((
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [0, 1, 0],
    [0, 1, 0],
    [1, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 0],
    [0, 1, 1],
    [1, 0, 0])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 1],
    [0, 1, 0],
    [0, 1, 0])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [0, 0, 1],
    [1, 1, 0],
    [0, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 0],
    [0, 1, 0],
    [1, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 1],
    [0, 1, 0],
    [1, 0, 0])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [1, 0, 1],
    [0, 1, 0],
    [0, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0])))

junctionsHitKernel.append(np.array((
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 1])))

junctionsMissKernel.append(np.array((
    [0, 0, 0],
    [0, 0, 0],
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

        #remove vertices in unknown area in ogm
        # voronoiVertices[map.ogm == -1] = 0

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
        visualization.visualizeVertices(map.frame_id,
                                2,
                                0,
                                ns,
                                0.1,
                                colors,
                                vizPoints,
                                False)

    def findVertices(self, voronoi):

        voronoi = voronoi.astype(int)
        voronoiEdges = []
        voronoiJunctions = []
        voronoiImage = np.zeros((voronoi.shape), np.uint8)
        voronoiImage[voronoi > 0] = 255

      #   for i in xrange(0, len(junctionsHitKernel)):
            # print junctionsHitKernel[i]
            # print junctionsMissKernel[i]
        # extract voronoi edges from every diffent structuring element
        for i in xrange(0, len(edgesHitKernel)) :
            voronoiEdges.append(ndimage.binary_hit_or_miss(voronoi,
            structure1 = edgesHitKernel[i], structure2 = edgesMissKernel[i]).astype(int))
 
        finalVoronoiEdges = voronoiEdges[0]
        for i in xrange(1, len(voronoiEdges)):
            finalVoronoiEdges = np.logical_or(finalVoronoiEdges,
                    voronoiEdges[i]).astype(int)

        finalVoronoiEdgesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        finalVoronoiEdgesImage[finalVoronoiEdges > 0] = 255
        
        # extract voronoi junctions from every diffent structuring element
        for i in xrange(0, len(junctionsHitKernel)):
            voronoiJunctions.append(ndimage.binary_hit_or_miss(voronoi,
                structure1 = junctionsHitKernel[i], structure2 = junctionsMissKernel[i]).astype(int))

        finalVoronoiJunctions = voronoiJunctions[0]
        for i in xrange(1, len(voronoiJunctions)):
            finalVoronoiJunctions = np.logical_or(finalVoronoiJunctions,
                    voronoiJunctions[i]).astype(int)

        finalVoronoiJunctionsImage = np.zeros((finalVoronoiJunctions.shape), np.uint8)
        finalVoronoiJunctionsImage[finalVoronoiJunctions > 0] = 255

        # create the final voronoi vertices array
        voronoiVertices = np.logical_or(finalVoronoiEdges,
                finalVoronoiJunctions).astype(int)

        voronoiVerticesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        voronoiVerticesImage[voronoiVertices > 0] = 255
    
      #   cv2.imshow('voronoi', voronoiImage)
        # cv2.imshow('voronoiEdges', finalVoronoiEdgesImage)
        # cv2.imshow('voronoiJunctions', finalVoronoiJunctionsImage)
        cv2.imshow("voronoiVertices", voronoiVerticesImage)
        cv2.waitKey(1000)

        return voronoiVertices

 
  
