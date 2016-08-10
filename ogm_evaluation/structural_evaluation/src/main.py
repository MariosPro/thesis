#!/usr/bin/env python

import time
import rospy
from structural_evaluation import StructuralEvaluation
# The main function of the program
if __name__ == '__main__':

    print "Waiting 5 seconds for initialization"
    time.sleep(5)

    # Initializes the ROS node
    rospy.init_node('structural_evaluation_node', anonymous = True)
    # Creates a StructuralEvaluation object
    se = StructuralEvaluation()
    # ROS waits for events
    rospy.spin()

