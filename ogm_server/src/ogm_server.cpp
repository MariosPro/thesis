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
    if (argc > 5)
    {
      ROS_ERROR("%s", USAGE);
      exit(-1);
    }

    if (argc == 5)
    {
      std::string fname(argv[1]);
      _mapServer.reset(new MapServer(fname, argv[2]));
      _mapServers.push_back(_mapServer);
    
    std::string fname1(argv[3]);
     _mapServer.reset(new MapServer(fname1, argv[4]));
     _mapServers.push_back(_mapServer);

    }
    _loadMapService = _nh.advertiseService(
      "/ogm_server/load_static_map", &Server::loadMapCallback, this);

    _loadExternalMapService = _nh.advertiseService(
      "/ogm_server/load_static_map_external", 
        &Server::loadExternalMapCallback, this);
  }

  /**
  @brief Service callback for loading the map
  @param req [ogm_msgs::LoadMap::Request&] The service request
  @param res [ogm_msgs::LoadMap::Response&] The service response
  @return bool
  **/
  bool Server::loadMapCallback(
    ogm_msgs::LoadMap::Request& req,
    ogm_msgs::LoadMap::Response& res) 
  {
/*    if (_mapServer) */
    //{
      //ROS_WARN("Map already loaded!");
      //return false;
    /*}*/
    _mapServer.reset(new MapServer(req.mapFile, true));

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
    /*if (_mapServer) {*/
      //ROS_WARN("Map already loaded!");
      //return false;
    /*}*/
    _mapServer.reset(new MapServer(req.map));

    return true;
  }
} // end of namespace ogm_server
