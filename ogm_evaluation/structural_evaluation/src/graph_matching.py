#!/usr/bin/env python
import timeit
from topological_graph import TopologicalGraph
from map import Map

#Class for performing the topological Graph Matching
class GraphMatching:
    
    #Constructor
    def __init__(self, map1, map2):
        self.map1 = Map(map1, "visualization_map1")
        self.map2 = Map(map2, "visualization_map2")
        self.graph1 = TopologicalGraph()
        self.graph2 = TopologicalGraph()

 
    def extractTopologicalGraphs(self):
        start_time = timeit.default_timer()
        self.graph1.extractTopologicalGraph(self.map1)
        elapsed = timeit.default_timer() -start_time
        print "Topological Graph execution time (ms): ", elapsed * 1000
        # self.graph2.extractTopologicalGraph(self.map2)
