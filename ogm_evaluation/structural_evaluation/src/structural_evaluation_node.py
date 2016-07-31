#!/usr/bin/env python

import rospy
from ogm_communications.srv import StructuralEvaluation
from ogm_communications.srv import StructuralEvaluationResponse
from graph_matching import GraphMatching

class StructuralEvaluation:
    def __init__(self):
        stucturalEval_serv =rospy.Service('structural_evaluation/map_evaluation', \
                StructuralEvaluation, self.structuralEvaluationCallback)

    def structuralEvaluationCallback(self, req):
        res = StructuralEvaluationResponse()

        #Create graph matching instance
        self.graphMatching = GraphMatching(req.map1, req.map2)
        return res

# The main function of the program
if __name__ == '__main__':
    # Initializes the ROS node
    rospy.init_node('structural_evaluation_node', anonymous = True)
    # Creates a StructuralEvaluation object
    se = StructuralEvaluation()
    # ROS waits for events
    rospy.spin()
 
