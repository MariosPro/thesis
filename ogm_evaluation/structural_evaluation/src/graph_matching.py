#!/usr/bin/env python
import timeit
import math
import numpy as np
import graph_tool.all as gt
from geometry_msgs.msg import Point
from alpha_star_search import Visitor
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
        touched_v = self.graph1.graph.new_vp("bool", val=False)
        touched_e = self.graph1.graph.new_ep("bool", val=False)
        pos = self.graph1.graph.vp['pose']
      
        for visitLim in range(4, self.graph1.graph.num_vertices()):
            target = self.graph1.graph.vertex(self.graph1.graph.num_vertices()-1)
            dist, pred = gt.astar_search(self.graph1.graph,
                                         self.graph1.graph.vertex(0),
                                         self.graph1.graph.ep['distance'],
                                         Visitor(touched_v,
                                                 touched_e,
                                                 visitLim),
                                         heuristic=lambda v: h(v, target, pos))
            filteredGraph1 = gt.GraphView(self.graph1.graph, vfilt=touched_v,
                                        efilt=touched_e)

            # for v in filteredGraph1.vertices():
             #    print "Vertice:", v 
                # for e in v.out_neighbours():
                    # print "edge ", v, " ", e

            subgraph1, vmap1, emap1 = GraphUtils.prune(filteredGraph1)

            print "subgraph1: V=", subgraph1.num_vertices(), "E=", \
                subgraph1.num_edges()

  #           print "subgraph1: " + GraphUtils.format_vlist(subgraph1)
            # print "S1 -> G1: " + GraphUtils.format_vmap(vmap1)
#             for v in subgraph1.vertices():
                # print "Vertice", v, subgraph1.vp.pose[v].a
                # for e in v.out_neighbours():
                    # print "edge ", v, " ", e

            vm = gt.subgraph_isomorphism(subgraph1,
                                         self.graph2.graph,
                                         induced=True, max_n = 10)
            if len(vm) > 0:
                print "found isomorphisms= ", len(vm), "number of vertices: ", visitLim
                vertexMapping = vm
                subgraph12 = subgraph1.copy()
                # break
        
        g = self.graph2.graph.copy()
        vMasks = []
        eMasks = []
        for i in range(len(vertexMapping)):
            g.set_vertex_filter(None)
            g.set_edge_filter(None)
            vmask, emask = gt.mark_subgraph(g, subgraph12, vertexMapping[i])
            vMasks.append(vmask)
            eMasks.append(emask)
    #         print ("VERTEX filter:", vmask.get_array())
            # print ("EDGE filter:", emask.get_array())
            g.set_vertex_filter(vmask)
            g.set_edge_filter(emask)
    
        for v in g.vertices():
            print "vertice:", v
            for e in v.out_neighbours():
                print "edge ", v, " ", e
        for v in subgraph12.vertices():
            print "sVertice", v, subgraph12.vp.pose[v].a
            for e in v.out_neighbours():
                print "sedge ", v, " ", e
        print "subgraph2: V=", g.num_vertices(), "E=", g.num_edges()
        print "subgraph1: V=", subgraph12.num_vertices(), "E=", \
            subgraph12.num_edges()

        subgraphPoses2 = []
        for v in g.vertices():
            subgraphPoses2.append(g.vp.pose[v].a)
        
        subgraphPoints2 = []
        for pose in subgraphPoses2:
            p = Point()
            p.x = pose[0]
            p.y = pose[1]
            subgraphPoints2.append(p)
      
        subgraphPoses1 = []
        for v in subgraph12.vertices():
                subgraphPoses1.append(subgraph12.vp.pose[v].a)
        
        subgraphPoints1 = []
        for pose in subgraphPoses1:
            p = Point()
            p.x = pose[0]
            p.y = pose[1]
            subgraphPoints1.append(p)
        print "subgraph2 points: ", len(subgraphPoints2)
        print "subgraph1 points: ", len(subgraphPoints1)

        return subgraphPoints1, subgraphPoints2

#     def graphMatching(self):
        # print "graph1 vertices", self.graph1.graph.num_vertices()

        # print "graph1 edges", self.graph1.graph.num_edges()

        # print "graph2 vertices", self.graph2.graph.num_vertices()

        # print "graph2 edges", self.graph2.graph.num_edges()
        # touched_v = self.graph1.graph.new_vp("bool", val=False)
        # touched_e = self.graph1.graph.new_ep("bool", val=False)
        # pos = self.graph1.graph.vp['pose']
        # # for j in range(0, self.graph1.graph.num_vertices() - 1):
            # # print "start vertex=", j
        # vmaps = []
        # for visitLim in range(4, self.graph1.graph.num_vertices()):
            # target = self.graph1.graph.vertex(self.graph1.graph.num_vertices()-1)
            # dist, pred = gt.astar_search(self.graph1.graph,
                                         # self.graph1.graph.vertex(0),
                                         # self.graph1.graph.ep['distance'],
                                         # Visitor(touched_v,
                                                 # touched_e,
                                                 # visitLim),
                                         # heuristic=lambda v: h(v, target, pos))
            # subgraphView = gt.GraphView(self.graph1.graph, vfilt=touched_v,
                                        # efilt=touched_e)
     # #        for v in self.graph1.graph.vertices():
                # # print touched_v[v]
            # # for v in self.graph1.graph.edges():
                # # print touched_e[v]

            # subgraph, vmap1, emap1 = GraphUtils.prune(subgraphView)
            # # for v in subgraph.vertices():
                # # x = subgraph.vp.pose[v].a
                # # print "pose", x
# #             subgraph.set_vertex_filter(touched_v)
              # # subgraph.set_edge_filter(touched_e)
            # subgraph.list_properties()
            # adj = gt.adjacency(subgraph)
            # adj = np.transpose(adj.toarray())
            # # print adj
            # # print adj.toarray()
            # print "subgraph vertices", subgraph.num_vertices()
            # print "subgrpah: " + GraphUtils.format_vlist(subgraph)
            # print "subgraph edges", subgraph.num_edges()
               # # # edge_weights[e] = adj[i,j]
            # for v in subgraph.vertices():
                # print "Vertice:", v
                # for e in v.out_neighbours():
                    # print "edge ", v, " ", e

  # #           for v in subgraph.vertices():
                # # # print subgraph.vp.pose[v]

            # s = gt.Graph(directed =False)
            # # s = subgraph.copy()
            # # vlist = s.add_vertex(subgraph.num_vertices())
            # # s.add_edge_list(np.transpose(adj.nonzero()))
            # s.add_vertex(adj.shape[0])
            # num_vertices = adj.shape[0]
            # for i in range(num_vertices - 1):
                # for j in range(i + 1, num_vertices):
                    # if adj[i, j] != 0:
                        # e = s.add_edge(i, j)
                        # # # edge_weights[e] = adj[i,j]
            # print "s vertices", s.num_vertices()
            # print "s edges", s.num_edges()

            # for v in s.vertices():
                    # print "SVertice:", v
                    # for e in v.out_neighbours():
                        # print "Sedge ", v, " ", e
            # vm = gt.subgraph_isomorphism(s, self.graph2.graph, induced=True, max_n = 1)
            # if len(vm) > 0:
                # print "found isomorphisms= ", len(vm), "number of vertices: ", visitLim
                # subgraph1 = gt.Graph(subgraph)
                # s1 = s.copy()
                # vmaps.append(vm)
            # else:
                # break
        # g = self.graph2.graph.copy()
        # vm = vmaps[len(vmaps)-1]
        # print len(vmaps)
        # print vmaps
        # for i in range(len(vm)):
            # g.set_vertex_filter(None)
            # g.set_edge_filter(None)
            # vmask, emask = gt.mark_subgraph(g, s1, vm[i])
      # #       print vm[i].a
            # print ("VERTEX filter:", vmask.get_array())
            # print ("EDGE filter:", emask.get_array())
            # g.set_vertex_filter(vmask)
            # g.set_edge_filter(emask)
            # # print "similarity", gt.similarity(g, subgraph1)
            # # print "similarity", gt.similarity(g, s1)
        # for v in g.vertices():
            # print "vertice:", v
            # for e in v.out_neighbours():
                # print "edge ", v, " ", e


        # subgraphPoses2 = []
        # for v in g.vertices():
            # subgraphPoses2.append(g.vp.pose[v].a)
    # #     print type(subgraphPoses)
        # # print subgraphPoses
        # subgraphPoints2 = []
        # for pose in subgraphPoses2:
            # p = Point()
            # p.x = pose[0]
            # p.y = pose[1]
            # subgraphPoints2.append(p)
      
        # subgraphPoses1 = []
        # for v in subgraph1.vertices():
                # subgraphPoses1.append(subgraph1.vp.pose[v].a)
        # subgraphPoints1 = []
        # print subgraphPoses1
        # for pose in subgraphPoses1:
            # p = Point()
            # p.x = pose[0]
            # p.y = pose[1]
            # subgraphPoints1.append(p)
        
        # print "subgraph1 vertices", subgraph1.num_vertices()
        # print "subgraph1 vertices", s1.num_vertices()
        # print "subgraph2 vertices", g.num_vertices()

        # print len(subgraphPoints2)
        # print len(subgraphPoints1)
        # return subgraphPoints1, subgraphPoints2
        # # g.list_properties()

# #
