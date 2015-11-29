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

#include "ogm_server/map_loader.h"
#include <ogm_msgs/LoadExternalMap.h>

#define USAGE "USAGE: load_map <map_file.yaml> true/false (true for ground truth)"

/**
@brief Main function of the server node
@param argc [int] Number of input arguments
@param argv [char**] Input arguments
@return int
**/
int main(int argc, char** argv) {

  ros::init(argc, argv, "map_loader", ros::init_options::AnonymousName);

  ros::NodeHandle nh;

  if (argc == 3) {
    ros::ServiceClient client;

    while (!ros::service::waitForService(
      "/map_server/load_static_map_external", ros::Duration(.1)) && ros::ok()) 
    {
      ROS_WARN(
        "Trying to register to /ogm_server/load_static_map_external...");
    }
    client = nh.serviceClient<ogm_msgs::LoadExternalMap>
      ("/ogm_server/load_static_map_external", true);

    ogm_msgs::LoadExternalMap srv;

    if(argv[2])
      srv.request.groundTruth = true;
    else
      srv.request.groundTruth = false;

    srv.request.mapFile = std::string(argv[1]);

    if (client.call(srv)) {
      ROS_INFO("Map successfully loaded");
      return 0;
    }
    else {
      ROS_ERROR("Could not load map, maybe already loaded...");
      return -1;
    }

  }
  else {
    ROS_ERROR("%s", USAGE);
    return -1;
  }
}
