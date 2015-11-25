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

#include "ogm_server/map_server.h"

namespace ogm_server {

  /**
  @brief Constructor by filename
  @param fname [const std::string&] The file name
  @return void
  **/
  MapServer::MapServer(const std::string& fname, bool ground_truth)
  {

    map_.ground_truth = ground_truth;
    ROS_INFO_STREAM("ground_truth=" << ground_truth);

    map_.map = map_loader::loadMap(fname);
 
    meta_data_message_ = map_.map.info;

    publishData();  
  }

  /**
  @brief Constructor by occupancy grid map
  @param map [const ogm _msgs::MapMsg&] The occupancy grid map
  @return void
  **/
  MapServer::MapServer(const ogm_msgs::MapMsg& map) 
  {

    map_ = map;

    meta_data_message_ = map_.map.info;

    publishData();
  }

  /**
  @brief Publishes the map data and metadata
  @return void
  **/
  void MapServer::publishData(void) 
  {
    ROS_INFO_STREAM("PUB");
    ROS_INFO_STREAM(" "<< map_.ground_truth <<" " 
                        <<map_.map.info.width << " " <<
                    map_.map.info.height << " " <<
                    map_.map.info.resolution << " " << 
                    map_.map.info.origin.position.x << " " <<
                    map_.map.info.origin.position.y 
                    );


    tfTimer = n.createTimer(ros::Duration(0.1), 
      &MapServer::publishTransform, this);

    //!< Latched publisher for metadata
    metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1000);
    metadata_pub.publish( meta_data_message_ );

    //!< Latched publisher for data
    map_pub = n.advertise<ogm_msgs::MapMsg>("map", 1000, true);
    map_pub.publish( map_ );
  }

  /**
  @brief Publishes the map to map_static transform
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void MapServer::publishTransform(const ros::TimerEvent&) {
    
    tf::Vector3 translation(
      map_.map.info.origin.position.x, 
      map_.map.info.origin.position.y, 
      0);
    
    tf::Quaternion rotation;
    
    rotation.setRPY(0, 0, tf::getYaw(map_.map.info.origin.orientation));

    tf::Transform worldTomap(rotation, translation);

    tfBroadcaster.sendTransform(
      tf::StampedTransform(worldTomap, ros::Time::now(), "map", "map_static"));
    
  }

} // end of namespace ogm_server

