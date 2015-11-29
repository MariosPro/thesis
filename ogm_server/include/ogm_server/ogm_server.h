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
/*#include <stdio.h>*/
/*#include <stdlib.h>*/
#include <vector>
#include <ros/ros.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <ogm_server/map_server.h>
#include <ogm_msgs/MapsMsg.h>
#include <ogm_msgs/LoadMap.h>
#include <ogm_msgs/LoadMaps.h>
#include <ogm_msgs/LoadExternalMap.h>

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

      /**
      @brief Publishes the map data and metadata
      @return void
      **/
      void publishData();

      /**
      @brief Publishes the map to map_static transform
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/
      void publishTransform(const ros::TimerEvent& ev);


      //!< Services --------------------------

      /**
      @brief Service callback for loading both default maps
      @param req [ogm_msgs::LoadMaps::Request&] The service request
      @param res [ogm_msgs::LoadMaps::Response&] The service response
      @return bool
      **/
      bool loadMapsCallback(ogm_msgs::LoadMaps::Request& req,
        ogm_msgs::LoadMaps::Response& res);

      /**
      @brief Service callback for loading the map
      @param req [ogm_msgs::LoadMap::Request&] The service request
      @param res [ogm_msgs::LoadMap::Response&] The service response
      @return bool
      **/
      bool loadMapCallback(ogm_msgs::LoadMap::Request& req,
        ogm_msgs::LoadMap::Response& res);

      /**
      @brief Service callback for loading the map from GUI
      @param req [ogm_msgs::LoadExternalMap::Request&] The service request
      @param res [ogm_msgs::LoadExternalMap::Response&] The service response
      @return bool
      **/
      bool loadExternalMapCallback(ogm_msgs::LoadExternalMap::Request& req,
        ogm_msgs::LoadExternalMap::Response& res);

    private:

      //!< THe ROS node handle
      ros::NodeHandle _nh;
      //!< A pointer to a groundTruth Map object
      MapServerPtr _groundTruthMap;
      //!< A pointer to a  slam Map object
      MapServerPtr _slamMap;
      //!< Service server for loading maps from files
      ros::ServiceServer _loadMapService;
      //!< Service server for loading maps from files
      ros::ServiceServer _loadMapsService;
      //!< Service server for loading maps from GUI
      ros::ServiceServer _loadExternalMapService;
      //!< ROS publisher for posting the map
      ros::Publisher map_pub;
      //!< ROS publisher for posting the map metadata
      ros::Publisher metadata_pub;
      //!< ROS timer for tf posting
      ros::Timer tfTimer;
      //!< ROS tf broadcaster
      tf::TransformBroadcaster tfBroadcaster;
      //!< The Maps Msg
      ogm_msgs::MapsMsg _maps;
  };
}


#endif
