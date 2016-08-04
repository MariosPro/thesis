#!/usr/bin/env python

import rospy
from ogm_communications.srv import ServerRequestStructuralEvaluation
from ogm_communications.srv import ServerRequestStructuralEvaluationResponse
from graph_matching import GraphMatching

class StructuralEvaluation:

     # Constructor
    def __init__(self):

        print "Created StructuralEvaluation instance"
        structuralEval = rospy.Service('/structural_evaluation/map_evaluation', \
                ServerRequestStructuralEvaluation, self.callback)

    def callback(self, req):
        res = ServerRequestStructuralEvaluationResponse()

        #Create graph matching instance
        self.graphMatching = GraphMatching(req.map1, req.map2)
        
        #Extract the TopologicalGraphs
        self.graphMatching.extractTopologicalGraphs()

        return res


