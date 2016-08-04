#!/usr/bin/env python

from voronoi_diagram import VoronoiDiagram

#Class for extracting Topological Graph from OGM
class TopologicalGraph:
    
    #Constructor
    def __init__(self):
        self.voronoiDiagram = VoronoiDiagram()

    def extractTopologicalGraph(self, map):
        self.voronoi = self.voronoiDiagram.extractVoronoi(map)
    

