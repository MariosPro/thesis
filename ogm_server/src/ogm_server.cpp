/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2015 OGM Validator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

   Authors :
   * Marios Protopapas, protopapasmarios@gmail.com
   * Manos Tsardoulias, etsardou@gmail.com
******************************************************************************/
#include <ogm_server/ogm_server.h>

namespace ogm_server
{

  /**
  @class Server
  @brief Implements the OGM server functionalities
  **/ 
  Server::Server(int argc, char** argv) 
  {
    if (argc > 3)
    {
      ROS_ERROR("%s", USAGE);
      exit(-1);
    }

    if (argc == 3)
    {
      std::string fname(argv[1]);
      std::string fname1(argv[2]);
      _groundTruthMap.reset(new MapServer(fname));
      _slamMap.reset(new MapServer(fname1));
    }

    _maps.groundTruthMap = _groundTruthMap->getMap();
    _maps.slamMap =  _slamMap->getMap();

    _loadExternalMapService = _nh.advertiseService(
      "/ogm_server/load_static_map_external", 
        &Server::loadExternalMapCallback, this);
 
    _loadExternalMapsService = _nh.advertiseService(
      "/ogm_server/load_static_maps_external", 
        &Server::loadExternalMapsCallback, this);

    _loadMapsService = _nh.advertiseService(
     "/ogm_server/load_maps", &Server::loadMapsCallback, this);

    _guiRequestEvaluationService = _nh.advertiseService(
     "/ogm_server/map_evaluation", &Server::guiRequestEvaluationCallback, this);

  }

  /**
  @brief Service callback for loading both default maps
  @param req [ogm_msgs::LoadMaps::Request&] The service request
  @param res [ogm_msgs::LoadMaps::Response&] The service response
  @return bool
  **/
  bool Server::loadMapsCallback(ogm_msgs::LoadMaps::Request& req,
        ogm_msgs::LoadMaps::Response& res)
  {
      res.groundTruthMap = _groundTruthMap->getMap();
      res.slamMap = _slamMap->getMap();
      ROS_INFO("Sending both Maps to GUI");

      return true;

  }

  /**
  @brief Service callback for loading the map from GUI
  @param req [ogm_msgs::LoadExternalMap::Request&] The service request
  @param res [ogm_msgs::LoadExternalMap::Response&] The service response
  @return bool
  **/
  bool Server::loadExternalMapCallback(
    ogm_msgs::LoadExternalMap::Request& req,
    ogm_msgs::LoadExternalMap::Response& res)
  {
    if(req.groundTruth)
    {
      _groundTruthMap.reset(new MapServer(req.mapFile));
      res.map = _groundTruthMap->getMap();
      std::cout << req.mapFile << std::endl;
    }
    else
    {
      _slamMap.reset(new MapServer(req.mapFile));
      res.map = _slamMap->getMap();
    }
    ROS_INFO("Sending Map to GUI");

    return true;
  }

  /**
  @brief Service callback for loading external both maps to server
  @param req [ogm_msgs::LoadExternalMaps::Request&] The service request
  @param res [ogm_msgs::LoadExternalMaps::Response&] The service response
  @return bool
  **/
  bool Server::loadExternalMapsCallback(
    ogm_msgs::LoadExternalMaps::Request& req,
    ogm_msgs::LoadExternalMaps::Response& res)
  {
    _groundTruthMap.reset(new MapServer(req.groundTruthMapFile));
    _slamMap.reset(new MapServer(req.slamMapFile));
    _maps.groundTruthMap = _groundTruthMap->getMap();
    _maps.slamMap =  _slamMap->getMap();
    
    ROS_INFO("Loading both Maps to server");


    return true;
  }

  /**
  @brief Service callback for map Evaluation request from GUI
  @param req [ogm_msgs::GuiRequestEvaluation::Request&] The service request
  @param res [ogm_msgs::GuiRequestEvaluation::Response&] The service response
  @return bool
  **/
  bool Server::guiRequestEvaluationCallback(
    ogm_msgs::GuiRequestEvaluation::Request& req,
    ogm_msgs::GuiRequestEvaluation::Response& res)
  {
    ros::ServiceClient client;
    ogm_msgs::ServerRequestEvaluation srv;

   while (!ros::service::waitForService
       ("/ogm_evaluation/map_evaluation", ros::Duration(.1)) &&
         ros::ok())
    {
       ROS_WARN
         ("Trying to register to /ogm_valuation/map_evaluation..");
    }

    client = _nh.serviceClient<ogm_msgs::ServerRequestEvaluation>
       ("/ogm_evaluation/map_evaluation", true);

    srv.request.method = req.method;
    srv.request.detector = req.detector;
    srv.request.descriptor = req.descriptor;
    srv.request.matcher = req.matcher;
    srv.request.matchingMethod = req.matchingMethod;
    srv.request.closestPointMethod = req.closestPointMethod;
    srv.request.distNorm = req.distNorm;
    srv.request.transform = req.transform;
    srv.request.groundTruthMap = _groundTruthMap->getMap();
    srv.request.slamMap = _slamMap->getMap();
    srv.request.matchingRatio = req.matchingRatio;
    srv.request.ransacReprjError = req.ransacReprjError;
    srv.request.scaleMapsBrushfire = req.scaleMapsBrushfire;
    srv.request.binary = req.binary;
    srv.request.manualAlignment = req.manualAlignment;
    srv.request.gaussianBlur1 = req.gaussianBlur1;
    srv.request.gaussianBlur2 = req.gaussianBlur2;
    srv.request.medianBlur1 = req.medianBlur1;
    srv.request.medianBlur2 = req.medianBlur2;
    srv.request.gaussianKernel1 = req.gaussianKernel1;
    srv.request.gaussianKernel2 = req.gaussianKernel2;
    srv.request.medianKernel1 = req.medianKernel1;
    srv.request.medianKernel2 = req.medianKernel2;

    if (client.call(srv)) 
    {
      ROS_INFO("[ogm_server] Map Evaluation metric succesfully completed");
      res.result =  srv.response.result;
      res.matchedImage = srv.response.matchedImage;
      res.mergedImage = srv.response.mergedImage;
    }
    else
    {
       ROS_ERROR("[ogm_server] Map Evaluation request failed...");
       return false;
    }

    return true;

  }
} // end of namespace ogm_server
