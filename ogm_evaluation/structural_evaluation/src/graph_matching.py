#!/usr/bin/env python
import timeit
from topological_graph import TopologicalGraph
from map import Map


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
        return voronoiPoints1, verticesPoints1, voronoiPoints2, verticesPoints2
 
