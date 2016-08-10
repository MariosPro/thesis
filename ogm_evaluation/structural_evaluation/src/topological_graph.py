#!/usr/bin/env python

import cv2
import numpy as np
from voronoi_diagram import VoronoiDiagram

#Class for extracting Topological Graph from OGM
class TopologicalGraph:
    
    #Constructor
    def __init__(self):
        self.voronoiDiagram = VoronoiDiagram()

    def extractTopologicalGraph(self, map):
        print "TopologicalGraph instance Created %s" %(map.frame_id)
        self.voronoi = self.voronoiDiagram.extractVoronoi(map)
        self.findVertices(self.voronoi)

    def findVertices(self, voronoi):
        edgesKernel = np.zeros((8, 3, 3), np.uint8)
        crossesKernel = np.zeros((8, 3, 3), np.uint8)
        crossesKernel[:, 1, 1] = 1
        edgesKernel[:, 1, 1] = 1

        edgesKernel[0, 0, 0] = 1
        edgesKernel[1, 0, 1] = 1
        edgesKernel[2, 0, 2] = 1
        edgesKernel[3, 1, 0] = 1
        edgesKernel[4, 1, 2] = 1
        edgesKernel[5, 2, 0] = 1
        edgesKernel[6, 2, 1] = 1
        edgesKernel[7, 2, 2] = 1
        # print edgesKernel

        crossesKernel[0, 0, 0] = 1
        crossesKernel[0, 1, 0] = 1
        crossesKernel[1, 1, 0] = 1
        crossesKernel[1, 2, 0] = 1
        crossesKernel[2, 0, 1] = 1
        crossesKernel[2, 0, 2] = 1
        crossesKernel[3, 0, 0] = 1
        crossesKernel[3, 0, 1] = 1
        crossesKernel[4, 0, 2] = 1
        crossesKernel[4, 1, 2] = 1
        crossesKernel[5, 1, 2] = 1
        crossesKernel[5, 2, 2] = 1
        crossesKernel[6, 2, 0] = 1
        crossesKernel[6, 2, 1] = 1
        crossesKernel[7, 2, 1] = 1
        crossesKernel[7, 2, 2] = 1
        # print crossesKernel

        print type(voronoi)
        voronoi = voronoi.astype(int)
        voronoiImage = np.zeros((voronoi.shape), np.uint8)
        voronoiImage[voronoi > 0] = 255
        # print voronoiImage.dtype
        # print voronoiImage
        cv2.imshow('voronoi', voronoiImage)
        cv2.waitKey(1000)
        
