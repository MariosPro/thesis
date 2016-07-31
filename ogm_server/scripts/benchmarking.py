#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import yaml
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
                       int(service_dict['binary']),
                       int(service_dict['manualAlignment']),
                       transform)
        return resp.result
    except rospy.ServiceException, e:
        print "Service ServerRequestEvaluation call failed: %s"%e


if __name__ == "__main__":
    service_params_keys = ['method', 'detector', 'descriptor', 'matcher',
        'matchingMethod', 'matchingRatio', 'ransacReprjError', 'distNorm',
        'x', 'y','theta','scale','slamOffsetScale', 'groundTruthOffsetscale', 'scaleMapsBrushfire', 'binary', 'manualAlignment']
    maps_path = rospkg.RosPack().get_path("ogm_resources") + '/maps/'
    maps_yaml_path = rospkg.RosPack().get_path("ogm_server") +'/scripts/maps.yaml'
    results_yaml_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/results.yaml'
    parameters_yaml_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/parameters.yaml'
    resultsFile = open(results_yaml_path, 'w')
    with open(maps_yaml_path, 'r') as f:
        maps_dict = yaml.load(f)
    with open(parameters_yaml_path, 'r') as p:
        parameters_dict = yaml.load(p)

    for maps, pair in maps_dict.iteritems():
        for item in pair:
            print "Find feature matching for this pair of maps: %s " % pair[item]
            groundTruthMapFile, slamMapFile = pair[item].split()
            groundTruthMapFile = maps_path + groundTruthMapFile + '.yaml'
            slamMapFile = maps_path + slamMapFile + '.yaml'
            callLoadExternalMapsService(groundTruthMapFile, slamMapFile)
            for parameters, comb in parameters_dict.iteritems():
                service_params_values = comb
                service_dict = dict(zip(service_params_keys,
                    service_params_values))
                result = callServerRequestEvaluationService(service_dict)
                resultsFile.write(str(result))
                resultsFile.write('\n')

