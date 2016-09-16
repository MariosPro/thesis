#!/usr/bin/env python
import timeit
import math
import operator
import numpy as np
import graph_tool.all as gt
from geometry_msgs.msg import Point
from alpha_star_search import BfsVisitor
from graph_utils import GraphUtils
from topological_graph import TopologicalGraph
from map import Map


def h(v, target, pos):
    return math.sqrt(sum((pos[v].a - pos[target].a) ** 2))


# Class for performing the topological Graph Matching
class GraphMatching:
    
    # Constructor
    def __init__(self, map1, map2):
        self.map1 = Map(map1, "visualization_map1")
        self.map2 = Map(map2, "visualization_map2")
        self.graph1 = TopologicalGraph()
        self.graph2 = TopologicalGraph()
 
    def extractTopologicalGraphs(self, parameters):
        start_time = timeit.default_timer()
        voronoiPoints1 = []
        verticesPoints1 = []
        voronoiPoints1, verticesPoints1 = self.graph1.extractTopologicalGraph(self.map1, parameters)
        elapsed = timeit.default_timer() - start_time
        print "Topological Graph 1 execution time (ms): ", elapsed * 1000

        voronoiPoints2 = []
        verticesPoints2 = []
        start_time = timeit.default_timer()
        voronoiPoints2, verticesPoints2 = self.graph2.extractTopologicalGraph(self.map2, parameters)
        elapsed = timeit.default_timer() - start_time
        print "Topological Graph 2 execution time (ms): ", elapsed * 1000
        # self.graphMatching()
        return voronoiPoints1, verticesPoints1, voronoiPoints2, verticesPoints2

    def graphMatching(self):
      #       for v in self.graph1.graph.vertices():
            # print "vertice:", v
            # for e in v.out_neighbours():
                # print "edge ", v, " ", e
     #    self.graph1.graph = gt.GraphView(self.graph1.graph, vfilt=lambda v:
                                         # v.out_degree() > 1)
        # self.graph2.graph = gt.GraphView(self.graph2.graph, vfilt=lambda v:
                                         # v.out_degree() > 1)

   #      touched_v = self.graph1.graph.new_vp("bool", val=False)
        # touched_e = self.graph1.graph.new_ep("bool", val=False)
        # pos = self.graph1.graph.vp['pose']
        vertexMapping = []
        # subgraph12 = gt.Graph(directed=False)
        candidateSubgraphs = []
        subgraphsVertices = []
        vM = []
        for i in xrange(0, self.graph1.graph.num_vertices()):
            touched_v = self.graph1.graph.new_vp("bool", val=False)
            touched_e = self.graph1.graph.new_ep("bool", val=False)
            maxSubgraphVertices = 0
            subgraph12 = gt.Graph(directed=False)
            for visitLim in xrange(4, self.graph1.graph.num_vertices()):
                # target = self.graph1.graph.vertex(self.graph1.graph.num_vertices()-1)
               #  dist, pred = gt.astar_search(self.graph1.graph,
                                             # self.graph1.graph.vertex(i),
                                             # self.graph1.graph.ep['distance'],
                                             # Visitor(touched_v,
                                                     # touched_e,
                                                     # visitLim),
                                             # heuristic=lambda v: h(v, target, pos))
                gt.bfs_search(self.graph1.graph, self.graph1.graph.vertex(i),
                              BfsVisitor(touched_v, touched_e, visitLim))
                filteredGraph1 = gt.GraphView(self.graph1.graph, vfilt=touched_v,
                                            efilt=touched_e)

                # for v in filteredGraph1.vertices():
                 #    print "Vertice:", v 
                    # for e in v.out_neighbours():
                        # print "edge ", v, " ", e

                subgraph1, vmap1, emap1 = GraphUtils.prune(filteredGraph1)

        #         print "subgraph1: V=", subgraph1.num_vertices(), "E=", \
                    # subgraph1.num_edges()

      #           print "subgraph1: " + GraphUtils.format_vlist(subgraph1)
                # print "S1 -> G1: " + GraphUtils.format_vmap(vmap1)
#             for v in subgraph1.vertices():
                    # print "Vertice", v, subgraph1.vp.pose[v].a
                    # for e in v.out_neighbours():
                        # print "edge ", v, " ", e

#                 vm = gt.subgraph_isomorphism(subgraph1,
                                             # self.graph2.graph,
                                             # vertex_label=(subgraph1.vp["exits"],
                                                         # self.graph2.graph.vp["exits"]),
                                             # induced=True)
                vm = gt.subgraph_isomorphism(subgraph1,
                                             self.graph2.graph,
                                             induced=True)

                if len(vm) > 0:
#                     print "found isomorphisms= ", len(vm), "start vertex: ", \
                        # i, "number of vertices: ", visitLim
                    if visitLim > maxSubgraphVertices:
                        vM = vm
                        subgraph12 = subgraph1.copy()
     #                    print "found max subgpraphs isomorphisms= ", len(vm), "start vertex: ", \
                        # i, "number of vertices: ", visitLim
            candidateSubgraphs.append(subgraph12)
            subgraphsVertices.append(subgraph12.num_vertices())
            vertexMapping.append(vM)
          
        g = self.graph2.graph.copy()
        vMasks = []
        eMasks = []
        maxSubgraph = max(enumerate(subgraphsVertices), key=operator.itemgetter(1))
        # maxVm = max(enumerate(vertexMapping), key = lambda tup: len(tup[1]))
        print maxSubgraph[0], maxSubgraph[1]
        vm = vertexMapping[maxSubgraph[0]]
        subgraph12 = candidateSubgraphs[maxSubgraph[0]]
        print "VertexMapping:", len(vm)
        for i in xrange(len(vm)):
            g.set_vertex_filter(None)
            g.set_edge_filter(None)
            vmask, emask = gt.mark_subgraph(g, subgraph12, vm[i])
            vMasks.append(vmask)
            eMasks.append(emask)
   #          print ("VERTEX filter:", vmask.get_array())
            # print ("EDGE filter:", emask.get_array())
            g.set_vertex_filter(vmask)
            g.set_edge_filter(emask)
          #   if i == 0:
                # break
            # assert gt.isomorphism(g, subgraph12)
            # candidateSubgraphs.append(g.copy())
    
      #   for v in g.vertices():
            # print "vertice:", v
            # for e in v.out_neighbours():
                # print "edge ", v, " ", e
        # for v in subgraph12.vertices():
            # print "sVertice", v, subgraph12.vp.pose[v].a
            # for e in v.out_neighbours():
                # print "sedge ", v, " ", e
        print "subgraph2: V=", g.num_vertices(), "E=", g.num_edges()
        print "subgraph1: V=", subgraph12.num_vertices(), "E=", \
            subgraph12.num_edges()
       #  for v in g.vertices():
            # print "subgraph2 angles: ", g.vp.angle[v].a
        # for v  in subgraph12.vertices():
            # print "subgraph1 angles: ", subgraph12.vp.angle[v].a
        # for v in g.vertices():
            # print "subgraph2 theta: ", g.vp.theta[v].a
        # for v in subgraph12.vertices():
            # print "subgraph1 theta: ", subgraph12.vp.theta[v].a

        subgraphPoses2 = [g.vp.pose[v].a.tolist() for v in g.vertices()]
        subgraphPoses1 = [subgraph12.vp.pose[v].a.tolist() for v in subgraph12.vertices()]

        return subgraphPoses1, subgraphPoses2 

