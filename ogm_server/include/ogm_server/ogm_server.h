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

#ifndef OGM_SERVER_H
#define OGM_SERVER_H

#define USAGE "\nUSAGE ogm_server <map.yaml> <ground_truth\n" \
              "  map.yaml: map description file\n"\
              " ground_truth: boolean var for ground truth load"

/*#include <stdio.h>*/
/*#include <stdlib.h>*/
#include <vector>
#include <ros/ros.h>
#include <ogm_server/map_server.h>
#include <ogm_msgs/LoadMap.h>
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
      
      //!< Services --------------------------
      
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
      //!< A pointer to a MapServe object
      MapServerPtr _mapServer;
      //!< A vector holding the 2 MapServe objects
      std::vector<MapServerPtr> _mapServers;
      //!< Service server for loading maps from files
      ros::ServiceServer _loadMapService;
      //!< Service server for loading maps from GUI
      ros::ServiceServer _loadExternalMapService;
            
      };
}


#endif
