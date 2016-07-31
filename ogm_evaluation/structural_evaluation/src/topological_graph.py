#!/usr/bin/env python

from voronoi_diagram import VoronoiDiagram

#Class for extracting Topological Graph from OGM
class TopologicalGraph:
    
    #Constructor
    def __init__(self, map):
        self.map = map
        self.VoronoiDiagram = VoronoiDiagram()

    def extractTopologicalGraph(map):
        self.voronoi = self.voronoiDiagram.extractVoronoi(map)
    

