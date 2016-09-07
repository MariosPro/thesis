#!/usr/bin/env python
import graph_tool.all as gt


class Visitor(gt.AStarVisitor):
    
    def __init__(self, touched_v, touched_e, visitLim):
        self.touched_v = touched_v
        self.touched_e = touched_e
        self.visited = 0
        self.visitLim = visitLim
        
    def discover_vertex(self, u):
        self.touched_v[u] = True
        self.visited += 1
    
    def examine_edge(self, e):
        self.touched_e[e] = True
    
    def edge_relaxed(self, e):
        if self.visited == self.visitLim:
            raise gt.StopSearch()
