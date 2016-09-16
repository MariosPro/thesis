#!/usr/bin/env python

# import cv2
import timeit
import math
import numpy as np
from itertools import izip
import mahotas as mh
from scipy import ndimage
from voronoi_diagram import VoronoiDiagram
from visualization import Visualization
from geometry_msgs.msg import Point
from graph_tool import *
from graph_tool import util


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
        
        # detect voronoi vertices
        start_time = timeit.default_timer()
        voronoiVertices = self.findVertices2(self.voronoi)
        elapsed = timeit.default_timer() - start_time
        print "Vertices execution time (ms): ", elapsed * 1000
        
        # detect vertices neighbors (in order to construct the topology graph)
        start_time = timeit.default_timer()
        verticesPoses = self.detectVerticesNeighbors(self.voronoi, voronoiVertices)
        elapsed = timeit.default_timer() - start_time
        print "Neighbors execution time (ms): ", elapsed * 1000
        
        # construct the Graph
        start_time = timeit.default_timer()
        self.graph.vp['pose'] = self.graph.new_vertex_property("vector<double>")
        self.graph.ep['distance'] = self.graph.new_edge_property("double")
        self.graph.vp['exits'] = self.graph.new_vertex_property("int")
        self.graph.add_vertex(len(verticesPoses))
        self.graph.add_edge_list(np.asarray(self.edges))
        for v1, v2 in izip(self.graph.vertices(), verticesPoses):
            self.graph.vp.pose[v1] = v2
        for e, d in izip(self.graph.edges(), self.distance):
            self.graph.ep.distance[e] = d
        for v1, v2 in izip(self.graph.vertices(), self.exits):
            self.graph.vp.exits[v1] = v2
        elapsed = timeit.default_timer() - start_time
        print "Graph construction execution time (ms): ", elapsed * 1000
        print "Vertices=", self.graph.num_vertices()
        print "Edges= ", self.graph.num_edges()
        # add angle of edges as graph edge property
        # self.findAngleOfEdges()

 #        print "edges"
        # for e in self.graph.edges():
            # print e.source(), " ", e.target()
        # print "vertices"
        # for v in self.graph.vertices():
            # print v
        # print "poses"
        # for v in self.graph.vertices():
            # print self.graph.vp.pose[v].a

        # print "exits"
        # for v in self.graph.vertices():
            # print self.graph.vp.exits[v]

    #     for v in self.graph.vertices():
            # print self.graph.vp.angleDiff[v].a
            # print self.graph.vp.theta[v].a

        # # for e in self.graph.edges():
            # print self.graph.ep.distance[e]

 #        print self.graph.vp.angle.get_2d_array(range(0,
                                                     # self.graph.num_vertices()))
        # print self.graph.vp.theta.get_2d_array(range(0,
                                                     # self.graph.num_vertices()))

        verticesPoints = np.argwhere(voronoiVertices == 1).tolist()   
       
       # visualize in rviz
        # vizPoints = []
      #   for i in xrange(voronoiVertices.shape[0]):
            # for j in xrange(voronoiVertices.shape[1]):
                # if voronoiVertices[i][j] == 1:
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
           
        return voronoiPoints, verticesPoints
      
    def endPoints(self, skel):
        endpoint1=np.array([[1, 0, 0],[0, 1, 0],[0, 0, 0]])
        endpoint2=np.array([[0, 1, 0],[0, 1, 0],[0, 0, 0]])
        endpoint3=np.array([[0, 0, 1],[0, 1, 0],[0, 0, 0]])
        endpoint4=np.array([[0, 0, 0],[0, 1, 1],[0, 0, 0]])
        endpoint5=np.array([[0, 0, 0],[1, 1, 0],[0, 0, 0]])
        endpoint6=np.array([[0, 0, 0],[0, 1, 0],[1, 0, 0]])
        endpoint7=np.array([[0, 0, 0],[0, 1, 0],[0, 1, 0]])
        endpoint8=np.array([[0, 0, 0],[0, 1, 0],[0, 0, 1]])
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
 
    def junctions(self, skel):
        junction1=np.array([[2, 1, 2],[2, 1, 2],[1, 2, 1]])
        junction2=np.array([[1, 2, 2],[2, 1, 1],[1, 2, 2]])
        junction3=np.array([[1, 2, 1],[2, 1, 2],[2, 1, 2]])
        junction4=np.array([[2, 2, 1],[1, 1, 2],[2, 2, 1]])
        junction5=np.array([[1, 2, 2],[2, 1, 2],[1, 2, 1]])
        junction6=np.array([[1, 2, 1],[2, 1, 2],[1, 2, 2]])
        junction7=np.array([[1, 2, 1],[2, 1, 2],[2, 2, 1]])
        junction8=np.array([[2, 2, 1],[2, 1, 2],[1, 2, 1]])
        junction9=np.array([[2, 0, 1],[1, 1, 0],[2, 1, 2]])
        junction10=np.array([[2, 1, 2],[1, 1, 0],[2, 0, 1]])
        junction11=np.array([[2, 1, 2],[0, 1, 1],[1, 0, 2]])
        junction12=np.array([[1, 0, 2],[0, 1, 1],[2, 1, 2]])

        ep1=mh.morph.hitmiss(skel,junction1)
        ep2=mh.morph.hitmiss(skel,junction2)
        ep3=mh.morph.hitmiss(skel,junction3)
        ep4=mh.morph.hitmiss(skel,junction4)
        ep5=mh.morph.hitmiss(skel,junction5)
        ep6=mh.morph.hitmiss(skel,junction6)
        ep7=mh.morph.hitmiss(skel,junction7)
        ep8=mh.morph.hitmiss(skel,junction8)
        ep9=mh.morph.hitmiss(skel,junction9)
        ep10=mh.morph.hitmiss(skel,junction10)
        ep11=mh.morph.hitmiss(skel,junction11)
        ep12=mh.morph.hitmiss(skel,junction12)

        ep = ep1+ep2+ep3+ep4+ep5+ep6+ep7+ep8+ep9+ep10+ep11+ep12
        return ep


    def findVertices2(self, voronoi):
        voronoi = voronoi
        endpoints = self.endPoints(voronoi)
        junctions = self.junctions(voronoi)
        voronoiVertices = np.logical_or(endpoints,  junctions)
        return voronoiVertices

    def findVertices(self, voronoi):

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

        voronoi = voronoi.astype(int)
        voronoiEdges = []
        voronoiJunctions = []
     #    voronoiImage = np.zeros((voronoi.shape), np.uint8)
        # voronoiImage[voronoi > 0] = 255

        # extract voronoi edges from every diffent structuring element
        voronoiEdges = [ndimage.binary_hit_or_miss(voronoi, edgesKernel[i]).astype(int)
                        for i in xrange(0, len(edgesKernel))]
 
        finalVoronoiEdges = reduce(lambda x, y: x+y, voronoiEdges)

   #      finalVoronoiEdgesImage = np.zeros((finalVoronoiEdges.shape), np.uint8)
        # finalVoronoiEdgesImage[finalVoronoiEdges > 0] = 255

        # extract voronoi junctions from every diffent structuring element
        voronoiJunctions = [ndimage.binary_hit_or_miss(voronoi,
                            structure1  = junctionsHitKernel[i],
                            structure2 = junctionsMissKernel[i]).astype(int)
                            for i in xrange(0, len(junctionsHitKernel))]
    
        finalVoronoiJunctions = reduce(lambda x, y: x+y, voronoiJunctions)
    
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
     
    def findAngleOfEdges(self):
        self.graph.vp['angle'] = self.graph.new_vertex_property("vector<int>")
        self.graph.vp['theta'] = self.graph.new_vertex_property("vector<int>")
        self.graph.vp['angleDiff'] = self.graph.new_vertex_property("vector<int>")

        poses = []
        for v in self.graph.vertices():
            poses.append(self.graph.vp.pose[v])
        print type(poses[0])
        for v in self.graph.vertices():
            theta = []
            angle = []
            for nv in v.out_neighbours():
                dx = poses[self.graph.vertex_index[nv]][0] - \
                    poses[self.graph.vertex_index[v]][0]
                dy = poses[self.graph.vertex_index[nv]][1] - \
                    poses[self.graph.vertex_index[v]][1]
                t = int(math.atan2(dy, dx) * 180 / math.pi) + 180
                theta.append(t)
                # print "theta:", theta
                # a = 0
                if 0 < t <= 90:
                    a = 1
                elif 90 < t <= 180:
                    a = 2
                elif 180 < t <= 270:
                    a = 3
                elif 270 < t <= 360:
                    a = 4
        #         elif 240 < t <= 300:
                    # a = 5
                # elif 300 < t <= 360:
                    # a = 6
        #         elif 180 < t <= 210:
                    # a = 7
                # elif 210 < t <= 240:
                    # a = 8
                # elif 240 < t <= 270:
                    # a = 9
                # elif 270 < t <= 300:
                    # a = 10
                # elif 300 < t <= 330:
                    # a = 11
                # elif 330 < t <= 360:
                    # a = 12
                angle.append(a)

            thetadiff = []
            for i in xrange(len(theta)-1):
                thetadiff.append(abs(theta[i] - theta[i+1]))
            angleMaxMinDiff  = [max(thetadiff), min(thetadiff)]
            self.graph.vp['theta'][v] = theta
            self.graph.vp['angle'][v] = angle
            self.graph.vp['angleDiff'][v] = angleMaxMinDiff
        
  #       self.graph.edge_properties['angle'] = self.graph.new_edge_property("int")
        # self.graph.edge_properties['theta'] = self.graph.new_edge_property("int")

        # poses = [] 
        # for v in self.graph.vertices():
            # poses.append(self.graph.vp.pose[v].a)

        # for e in self.graph.edges():
            # dx = poses[self.graph.vertex_index[e.target()]][0] - \
                # poses[self.graph.vertex_index[e.source()]][0]
            # dy = poses[self.graph.vertex_index[e.target()]][1] - \
                # poses[self.graph.vertex_index[e.source()]][1]
            # theta = int(math.atan2(dy, dx) * 180 / math.pi) + 180
            # self.graph.ep['theta'][e] = theta
            # # print "theta:", theta
            # angle = 0
            # if 0 < theta <= 30:
                # angle = 1
            # elif 30 < theta <= 60:
                # angle = 2
            # elif 60 < theta <= 90:
                # angle = 3
            # elif 90 < theta <= 120:
                # angle = 4
            # elif 120 < theta <= 150:
                # angle = 5
            # elif 150 < theta <= 180:
                # angle = 6
            # elif 180 < theta <= 210:
                # angle = 7
            # elif 210 < theta <= 240:
                # angle = 8
            # elif 240 < theta <= 270:
                # angle = 9
            # elif 270 < theta <= 300:
                # angle = 10
            # elif 300 < theta <= 330:
                # angle = 11
            # elif 330 < theta <= 360:
                # angle = 12

            # self.graph.ep['angle'][e] = angle
  
    def detectVerticesNeighbors(self, isOnVoronoi, isVertice):
        verticesPoses = np.argwhere(isVertice == 1).tolist() 
        self.edges = []
        self.distance = []
        self.exits = []
        [self.findVertexNeighbours(v, verticesPoses,
                                   isOnVoronoi, isVertice) 
         for v in verticesPoses]
        return verticesPoses

    def findVertexNeighbours(self, v, verticesPoses, isOnVoronoi, isVertice):
        moves = [[-1, -1], [-1, 0], [-1, 1], [0, -1],
                 [0, 1], [1, -1], [1, 0], [1, 1]]
        brush = np.full(isVertice.shape, -1)
        brush[isVertice > 0] = 0
        current = []
        edges = []
        current.append(v)
        _next = []
        expanded = True
        counter = -1
        while expanded:
            expanded = False
            counter += 1
            for c in current:
                found = False
                for m in moves:
                    x = c[0] + m[0]
                    y = c[1] + m[1]
                    if x >= 0 and x < isVertice.shape[0] \
                       and y >= 0 and y < isVertice.shape[1]:
                        if brush[x][y] == -1 and isOnVoronoi[x][y] == 1:
                            brush[x][y] = counter + 1
                            _next.append([x, y])
                            expanded = True
                        if brush[x][y] == 0 and isVertice[x][y] == 1:
                            if not(v[0] == x and 
                                   v[1] == y):
                                # add this node as neighbor with
                                # distance as edge weight
                                p = [x, y]
                                foundNeigh = p
                                found = True
                                e = [verticesPoses.index(v), 
                                     verticesPoses.index(p)]
                                edges.append(e)
                                self.distance.append(counter + 1)
                            if len(edges) == 1:
                                self.exits.append(len(current))
                if found:
                    for n in _next:
                        for m in moves:
                            x = n[0] + m[0]
                            y = n[1] + m[1]
                            if x >= 0 and x < isVertice.shape[0] \
                              and y >= 0 and y < isVertice.shape[1]:
                                  if foundNeigh ==  [x, y]:
                                    _next.remove(n)
            current = _next
            _next = []
        self.edges.extend(edges)



