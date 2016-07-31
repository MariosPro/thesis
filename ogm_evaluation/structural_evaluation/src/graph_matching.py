#!/usr/bin/env python
from topological_graph import TopologicalGraph
from map import Map

#Class for performing the topological Graph Matching
class GraphMatching:
    
    #Constructor
    def __init__(self, map1, map2):
        self.map1 = Map(map1)
        self.map2 = Map(map2)
        self.graph1 = TopologicalGraph(self.map1)
        self.graph2 = Topologicalgraph(self.map2)
