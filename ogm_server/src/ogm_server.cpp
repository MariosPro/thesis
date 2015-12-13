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
    //publishData();

   /* _loadMapService = _nh.advertiseService(*/
      /*"/ogm_server/load_static_map", &Server::loadMapCallback, this);*/

    _loadExternalMapService = _nh.advertiseService(
      "/ogm_server/load_static_map_external", 
        &Server::loadExternalMapCallback, this);

    _loadMapsService = _nh.advertiseService(
     "/ogm_server/load_maps", &Server::loadMapsCallback, this);

    _guiRequestEvaluationService = _nh.advertiseService(
     "/ogm_server/map_evaluation", &Server::guiRequestEvaluationCallback, this);

  }

  /**
  @brief Service callback for loading the map
  @param req [ogm_msgs::LoadMap::Request&] The service request
  @param res [ogm_msgs::LoadMap::Response&] The service response
  @return bool
  **/
 /* bool Server::loadMapCallback(*/
    //ogm_msgs::LoadMap::Request& req,
    //ogm_msgs::LoadMap::Response& res) 
  //{
    //if(req.groundTruth)
    //{
      //_groundTruthMap.reset(new MapServer(req.mapFile));
      //res.map = _groundTruthMap.getMap();
    //}
    //else
    //{
      //_slamMap.reset(new MapServer(req.mapFile));
      //res.map = _slamMap.getMap();
    //}
    
    //ROS_INFO("Sending Map to GUI");

    //return true;
  /*}*/

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
  @brief Service callback for map Evaluation request form GUI
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
    srv.request.transform = req.transform;
    srv.request.groundTruthMap = _groundTruthMap->getMap();
    srv.request.slamMap = _slamMap->getMap();

    if (client.call(srv)) 
    {
      ROS_INFO("Map Evaluation metric succesfully completed");
      res.result =  srv.response.result ;
    }
    else
    {
       ROS_ERROR("Map Evaluation request failed...");
    }

    return true;

  }


  /**
  @brief Publishes the map data and metadata
  @return void
  **/
  void Server::publishData(void) 
  {
    /*ROS_INFO_STREAM("PUB");*/
    //ROS_INFO_STREAM(" "<< map_.ground_truth <<" " 
                        //<<map_.map.info.width << " " <<
                    //map_.map.info.height << " " <<
                    //map_.map.info.resolution << " " << 
                    //map_.map.info.origin.position.x << " " <<
                    //map_.map.info.origin.position.y 
                    /*);*/


    tfTimer = _nh.createTimer(ros::Duration(0.1), 
      &Server::publishTransform, this);

    //!< Latched publisher for metadata
    /*metadata_pub= _nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);*/
    /*metadata_pub.publish( meta_data_message_ );*/

    //!< Latched publisher for data
    map_pub = _nh.advertise<ogm_msgs::MapsMsg>("map", 1, true);
    map_pub.publish( _maps );
  }

  /**
  @brief Publishes the map to map_static transform
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void Server::publishTransform(const ros::TimerEvent&) {

    /*tf::Vector3 translation(*/
      //map_.info.origin.position.x, 
      //map_.info.origin.position.y, 
      //0);

    //tf::Quaternion rotation;

    //rotation.setRPY(0, 0, tf::getYaw(map_.info.origin.orientation));

    //tf::Transform worldTomap(rotation, translation);

    //tfBroadcaster.sendTransform(
      /*tf::StampedTransform(worldTomap, ros::Time::now(), "map", "map_static"));*/

  }

} // end of namespace ogm_server
