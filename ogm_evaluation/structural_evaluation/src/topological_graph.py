#!/usr/bin/env python

# import cv2
import timeit
import numpy as np
from scipy import ndimage
from voronoi_diagram import VoronoiDiagram
from visualization import Visualization
from geometry_msgs.msg import Point
from graph_tool import *
from graph_tool import util

# structuring elements for morphological operations
edgesKernel = []
edgesHitKernel = []
edgesMissKernel = []
junctionsHitKernel = []
junctionsMissKernel = []
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

# Class for extracting Topological Graph from OGM


class TopologicalGraph:

    # Constructor
    def __init__(self):
        self.voronoiDiagram = VoronoiDiagram()
        self.graph = Graph(directed = False)

    def extractTopologicalGraph(self, map, parameters):
        # visualization = Visualization(map.frame_id)
        print "TopologicalGraph instance Created %s" % (map.frame_id)
        
        # extract voronoi diagram
        start_time = timeit.default_timer()
        self.voronoi, voronoiPoints = self.voronoiDiagram.extractVoronoi(map, 
                                                                         parameters)
        elapsed = timeit.default_timer() - start_time
        print "Voronoi Diagram execution time (ms): ", elapsed * 1000
        # self.voronoi = np.array(([0,1,1,1],
                                # [0,1,0,0],
                                # [0,1,0,0],
                                # [0,1,1,1],
                                # [0,0,0,0]))
        # detect voronoi vertices
        voronoiVertices = self.findVertices(self.voronoi)
        # voronoiVertices = np.array(([0,0,0,1],
                                # [0,0,0,0],
                                # [0,0,0,0],
                                # [0,0,0,1],
                                # [0,1,0,0]))

        # detect vertices neighbors (in order to construct the topology graph)
        start_time = timeit.default_timer()
        self.detectVerticesNeighbors(self.voronoi, voronoiVertices)
        elapsed = timeit.default_timer() - start_time
        print "Neighbors execution time (ms): ", elapsed * 1000
        
        # visualize in rviz
        # vizPoints = []
        verticesPoints = []
        for i in xrange(voronoiVertices.shape[0]):
            for j in xrange(voronoiVertices.shape[1]):
                if voronoiVertices[i][j] == 1:
        #             p = Point()
                    # p.x = j
                    # p.y = map.height - i
                    verticesPoints.append([i, j])
#                     vp = Point()
                    # vp.x = j * map.resolution + map.origin['x']
                    # vp.y = (map.height - i) * map.resolution + map.origin['y']
                    # vizPoints.append(vp)
        # if map.frame_id == "visualization_map1":
            # ns = "map1/voronoiVertices"
        # else:
            # ns = "map2/voronoiVertices"
        # colors = [1.0, 0.0, 1.0, 0.0]
        # visualization.visualizeVertices(map.frame_id,
                                        # 2,
                                        # 0,
                                        # ns,
                                        # 0.1,
                                        # colors,
                                        # vizPoints,
                                        # False)
        # visualize the graph
        # for v in self.graph.vertices():
            # print  v
            # print "neighbors"
            # for n in v.all_neighbours():
                # print n
            # print "edges"
            # for e in v.all_edges():
                # print e

        # graph_draw(self.graph, pos=self.graph.vp['pose'],
                   # vertex_text= self.graph.vertex_index,
                   # edge_text = self.graph.ep['label'],
                   # edge_pen_width = self.graph.ep['distance'],
                   # output_size=[4024, 4024])
       
        return voronoiPoints, verticesPoints

    def findVertices(self, voronoi):

        voronoi = voronoi.astype(int)
        voronoiEdges = []
        voronoiJunctions = []
     #    voronoiImage = np.zeros((voronoi.shape), np.uint8)
        # voronoiImage[voronoi > 0] = 255

        # extract voronoi edges from every diffent structuring element
        for i in xrange(0, len(edgesKernel)):
            voronoiEdges.append(ndimage.binary_hit_or_miss(voronoi, edgesKernel[i]).astype(int))
 
        finalVoronoiEdges = voronoiEdges[0]
        for i in xrange(1, len(voronoiEdges)):
            finalVoronoiEdges = np.logical_or(finalVoronoiEdges,
                    voronoiEdges[i]).astype(int)

   #      finalVoronoiEdgesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        # finalVoronoiEdgesImage[finalVoronoiEdges > 0] = 255

        # extract voronoi junctions from every diffent structuring element
        for i in xrange(0, len(junctionsHitKernel)):
            voronoiJunctions.append(ndimage.binary_hit_or_miss(voronoi,
                structure1  = junctionsHitKernel[i], structure2 = junctionsMissKernel[i]).astype(int))

        finalVoronoiJunctions = voronoiJunctions[0]
        for i in xrange(1, len(voronoiJunctions)):
            finalVoronoiJunctions = np.logical_or(finalVoronoiJunctions,
                                                  voronoiJunctions[i]).astype(int)

#         finalVoronoiJunctionsImage = np.zeros((finalVoronoiJunctions.shape), np.uint8)
        # finalVoronoiJunctionsImage[finalVoronoiJunctions > 0] = 255

        # create the final voronoi vertices array
        voronoiVertices = np.logical_or(finalVoronoiEdges,
                                        finalVoronoiJunctions).astype(int)

  #       voronoiVerticesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        # voronoiVerticesImage[voronoiVertices > 0] = 255

        # cv2.imshow('voronoi', voronoiImage)
        # cv2.imwrite('/home/marios/Pictures/voronoi.jpg',voronoiImage)
        # cv2.imshow('voronoiEdges', finalVoronoiEdgesImage)
        # cv2.imshow('voronoiJunctions', finalVoronoiJunctionsImage)
        # cv2.imshow("voronoiVertices", voronoiVerticesImage)
        # cv2.waitKey(1000)

        return voronoiVertices

    def detectVerticesNeighbors(self, isOnVoronoi, isVertice):
      #   width = isVertice.shape[1]
        # height = isVertice.shape[0]
        verticesPoses = []

        for i in xrange(0, isVertice.shape[0]):
            for j in xrange(0, isVertice.shape[1]):
                if isVertice[i][j] == 1:
                    verticesPoses.append((i, j))

        self.graph.vertex_properties['pose'] = self.graph.new_vertex_property("vector<double>")
        self.graph.edge_properties['distance'] = self.graph.new_edge_property("double")
        # self.graph.edge_properties['label'] = self.graph.new_edge_property("string")

        for v in verticesPoses:        
            vertice = self.graph.add_vertex()
            self.graph.vertex_properties['pose'][vertice] = (v[0], v[1])
        for v in self.graph.vertices():
            # print "iteration ", v
            brush = np.full(isVertice.shape, -1)
            current = []
            brush[isVertice > 0] = 0
            # print brush
            current.append(self.graph.vertex_properties['pose'][v].a)
            # print self.graph.vertex_properties['pose'][v].a
            # print current
            # print type(current)
            _next = []
            expanded = True
            counter = -1
            while expanded:
                expanded = False
                counter += 1
                for c in current:
                    # print "current", c
                    for kx in range(-1, 2):
                        for ky in range(-1, 2):
                            if ky == 0 and kx == 0:
                                continue
                            x = c[0] + kx
                            y = c[1] + ky
                            if x >= 0 and x < isVertice.shape[0] \
                               and y >= 0 and y < isVertice.shape[1]:
                                # print "x,y =",x,y
                                if brush[x][y] == -1 and isOnVoronoi[x][y] == 1:
                                    brush[x][y] = counter + 1
                                    _next.append([x, y])
                                    expanded = True
                                    # print brush
                                if brush[x][y] == 0 and isVertice[x][y] == 1:
                                    v_pos = self.graph.vp['pose'][v].a
                                    # print "current vertex pos=",v_pos[0], v_pos[1]
                                    if not (v_pos[0] == x and v_pos[1] == y):
                                        # add this node as neighbor
                                        v1 = util.find_vertex(self.graph,
                                                        self.graph.vertex_properties['pose'],
                                                        (x, y))

                                        # print v, " ", v1[0]
                                        # add the edge (compute the distance)
                                        if not self.graph.edge(v, v1[0]):
                                            e = self.graph.add_edge(v, v1[0])
                                            
                                            # add distance as edge property
                                            self.graph.ep['distance'][e] = counter + 1
                                            # self.graph.ep.label[e] = self.graph.ep['distance'][e]
                                            # print "distance=", self.graph.ep['distance'][e]
                current = _next
                _next = []

