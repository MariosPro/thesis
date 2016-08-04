/******************************************************************************
 *
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

#ifndef OGM_SERVER_H
#define OGM_SERVER_H

#define USAGE "\nUSAGE ogm_server <groundTruthMap.yaml> <slamMap.yaml>\n" \
              "  groundTruthMap.yaml: map description file\n"\
              "  slamMap.yaml: map description file"
#include <vector>
#include <ros/ros.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <ogm_server/map_server.h>
#include <ogm_communications/MapsMsg.h>
//#include <ogm_communications/LoadMap.h>
#include <ogm_communications/LoadMapsFromServer.h>
#include <ogm_communications/LoadExternalMap.h>
#include <ogm_communications/LoadExternalMaps.h>
#include <ogm_communications/GuiRequestEvaluation.h>
#include <ogm_communications/ServerRequestEvaluation.h>
#include <ogm_communications/GuiRequestStructuralEvaluation.h>
#include <ogm_communications/ServerRequestStructuralEvaluation.h>

/**
@namespace ogm_server
@brief The main namespace for OGM Server
**/ 
namespace ogm_server {

  typedef boost::shared_ptr<MapServer> MapServerPtr;

  /**
  @class Server
  @brief Implements the OGM server functionalities
  **/ 
  class Server 
  {

    public:

      /**
      @brief Default constructor
      @param argc [int] Number of input arguments
      @param argv [char**] The input arguments
      @return void
      **/
      Server(int argc, char** argv);


      //!< Services --------------------------

      /**
      @brief Service callback for loading both default maps
      @param req [ogm_communications::LoadMapsFromServer::Request&] The service request
      @param res [ogm_communications::LoadMapsFromServer::Response&] The service response
      @return bool
      **/
      bool loadMapsCallback(ogm_communications::LoadMapsFromServer::Request& req,
        ogm_communications::LoadMapsFromServer::Response& res);

      /**
      @brief Service callback for loading the map
      @param req [ogm_communications::LoadMap::Request&] The service request
      @param res [ogm_communications::LoadMap::Response&] The service response
      @return bool
      **/
      /*bool loadMapCallback(ogm_communications::LoadMap::Request& req,*/
        //ogm_communications::LoadMap::Response& res);

      /**
      @brief Service callback for loading the map from GUI
      @param req [ogm_communications::LoadExternalMap::Request&] The service request
      @param res [ogm_communications::LoadExternalMap::Response&] The service response
      @return bool
      **/
      bool loadExternalMapCallback(ogm_communications::LoadExternalMap::Request& req,
        ogm_communications::LoadExternalMap::Response& res);
 
      /**
      @brief Service callback for loading external both Maps to server
      @param req [ogm_communications::LoadExternalMap::Request&] The service request
      @param res [ogm_communications::LoadExternalMap::Response&] The service response
      @return bool
      **/
      bool loadExternalMapsCallback(ogm_communications::LoadExternalMaps::Request& req,
        ogm_communications::LoadExternalMaps::Response& res);

      /**
      @brief Service callback for map Evaluation request form GUI
      @param req [ogm_communications::GuiRequestEvaluation::Request&] The service request
      @param res [ogm_communications::GuiRequestEvaluation::Response&] The service response
      @return bool
      **/
      bool guiRequestEvaluationCallback(ogm_communications::GuiRequestEvaluation::Request& req,
        ogm_communications::GuiRequestEvaluation::Response& res);

      /**
      @brief Service callback for structural Evaluation request from GUI
      @param req [ogm_communications::GuiRequestStructuralEvaluation::Request&] The service request
      @param res [ogm_communications::GuiRequestStructuralEvaluation::Response&] The service response
      @return bool
      **/
      bool guiRequestStructuralEvaluationCallback(
        ogm_communications::GuiRequestStructuralEvaluation::Request& req,
        ogm_communications::GuiRequestStructuralEvaluation::Response& res);

    private:

      //!< THe ROS node handle
      ros::NodeHandle _nh;

      //!< A pointer to a groundTruth Map object
      MapServerPtr _groundTruthMap;

      //!< A pointer to a  slam Map object
      MapServerPtr _slamMap;

      //!< Service server for loading maps from files 
      //ros::ServiceServer _loadMapService;
      
      //!< Service server for loading maps from files
      ros::ServiceServer _loadMapsService;
      
      //!< Service server for loading maps from GUI
      ros::ServiceServer _loadExternalMapService;
      
      //!< Service server for loading external both maps 
      ros::ServiceServer _loadExternalMapsService;
      
      //!< Service server for mapEvaluation
      ros::ServiceServer _guiRequestEvaluationService;
      
      //!< Service server for mapEvaluation
      ros::ServiceServer _guiRequestStructuralEvaluationService;

      //!< ROS publisher for posting the map
      ros::Publisher map_pub;
      
      //!< ROS publisher for posting the map metadata
      ros::Publisher metadata_pub;
      
      //!< ROS timer for tf posting
      ros::Timer tfTimer;
      
      //!< ROS tf broadcaster
      tf::TransformBroadcaster tfBroadcaster;
      
      //!< The Maps Msg
      ogm_communications::MapsMsg _maps;
  };
}


#endif
