#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import json
import yaml
import cv2
from collections import OrderedDict
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ogm_communications.srv import LoadExternalMaps
from ogm_communications.srv import GuiRequestEvaluation
from ogm_communications.msg import MapPose

def callLoadExternalMapsService(groundTruthMapFile, slamMapFile):
    rospy.wait_for_service('/ogm_server/load_static_maps_external')
    try:
        load_maps = rospy.ServiceProxy('/ogm_server/load_static_maps_external', LoadExternalMaps)
        resp = load_maps(groundTruthMapFile, slamMapFile)
    except rospy.ServiceException, e:
        print "Service LoadExternalMaps call failed: %s"%e

def callServerRequestEvaluationService(service_dict):
    rospy.wait_for_service('/ogm_server/map_evaluation')
    try:
        reqEval = rospy.ServiceProxy('ogm_server/map_evaluation', GuiRequestEvaluation)
        transform = MapPose()
        transform.pose.x = float(service_dict['x'])
        transform.pose.y = float(service_dict['y'])
        transform.pose.theta = float(service_dict['theta'])
        transform.scale = float(service_dict['scale'])
        transform.slamOffsetScale = float(service_dict['slamOffsetScale'])
        transform.groundTruthOffsetScale = float(service_dict['groundTruthOffsetscale'])

        resp = reqEval(service_dict['method'], 
                       service_dict['detector'], 
                       service_dict['descriptor'],
                       service_dict['matcher'],
                       service_dict['matchingMethod'],
                       ' ', service_dict['distNorm'],
                       float(service_dict['matchingRatio']),
                       float(service_dict['ransacReprjError']),
                       int(service_dict['scaleMapsBrushfire']),
                       int(service_dict['morphologicalFiltering']),
                       int(service_dict['binary']),
                       int(service_dict['manualAlignment']),
                       int(service_dict['gaussianBlur1']),
                       int(service_dict['gaussianBlur2']),
                       int(service_dict['medianBlur1']),
                       int(service_dict['medianBlur2']),
                       int(service_dict['gaussianKernel1']),
                       int(service_dict['gaussianKernel2']),
                       int(service_dict['medianKernel1']),
                       int(service_dict['medianKernel2']),
                       transform)
        return resp.result, resp.matches, resp.acceptance,resp.quality,\
                resp.overlapArea, resp.meanExTime, resp.initialMatchedImage, resp.finalMatchedImage, resp.mergedImage
    except rospy.ServiceException, e:
        print "Service ServerRequestEvaluation call failed: %s"%e


if __name__ == "__main__":
    maps_path = rospkg.RosPack().get_path("ogm_resources") + '/maps/'
    maps_yaml_path = rospkg.RosPack().get_path("ogm_server") +'/scripts/maps.yaml'
    results_path =rospkg.RosPack().get_path("ogm_server") + \
            '/scripts/benchmarking_results/merging/map3/ALL/results.json'
    parameters_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/all_parameters.json'
    images_path =rospkg.RosPack().get_path("ogm_server") + \
            "/scripts/benchmarking_results/merging/map3/ALL/"
    image1= "initialMatchedImage"
    image2 = "finalMatchedImage"
    image3 = "mergedImage"
    resultsFile = open(results_path, 'w')
    with open(maps_yaml_path, 'r') as f:
        maps_dict = yaml.load(f)
    with open(parameters_path, 'r') as p:
        parameters_list = json.load(p, object_pairs_hook=OrderedDict)
    print type(parameters_list)

    results = list()
    count = 1
    bridge = CvBridge()
    # resultsFile.write("[")
    resultsFile.write("\n")
    for maps, pair in maps_dict.iteritems():
        for item in pair:
            print "Find feature matching for this pair of maps: %s " % pair[item]
            groundTruthMapFile, slamMapFile = pair[item].split()
            groundTruthMapFile = maps_path + groundTruthMapFile + '.yaml'
            slamMapFile = maps_path + slamMapFile + '.yaml'
            callLoadExternalMapsService(groundTruthMapFile, slamMapFile)
            for parameters in parameters_list:
                print parameters
               #  if count > 1072:
                result, matches, acceptance, quality, overlapArea, meanExTime, initialMatchedImage,finalMatchedImage, mergedImage = callServerRequestEvaluationService(parameters)
               #  cv_image1 = bridge.imgmsg_to_cv2(initialMatchedImage, "bgr8")
                cv_image2 = bridge.imgmsg_to_cv2(finalMatchedImage, "bgr8")
                cv_image3 = bridge.imgmsg_to_cv2(mergedImage, "mono8")
                image1_path = images_path + image1 + str(count) +".png"
                image2_path = images_path + image2 + str(count) +".png"
                image3_path = images_path + image3 + str(count) +".png"
             #    cv2.imwrite(image1_path, cv_image1)
                cv2.imwrite(image2_path, cv_image2)
                cv2.imwrite(image3_path, cv_image3)
                results_dict = parameters
                results_dict["OMSE"] = result
                results_dict["matches"] = matches
                results_dict["acceptance"] = acceptance
                results_dict["quality"] = quality
                results_dict["overlapAera"] = overlapArea
                results_dict["meanExTime"] = meanExTime
# print json.dumps(results, sort_keys=False, indent=4)
                resultsFile.write(json.dumps(results_dict, sort_keys=False,
                    indent=4))
                if count != len(parameters_list):
                    resultsFile.write(",")
                    resultsFile.write("\n")
                count = count +1
#                 if count == 3:
                    # break
resultsFile.write("\n")
resultsFile.write("]")
resultsFile.close()
   
