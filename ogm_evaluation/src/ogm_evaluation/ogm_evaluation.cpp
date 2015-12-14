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
******************************************************************************/
#include "ogm_evaluation/ogm_evaluation.h"

namespace ogm_evaluation
{
  /**
  @class MapEvaluator
  @brief Implements the OGM evaluations methods
  **/ 
  MapEvaluator::MapEvaluator(int argc, char** argv)
  {
    _mapEvaluationService =  _nh.advertiseService(
        "ogm_evaluation/map_evaluation", &MapEvaluator::evaluationCallback, this);

  }

  /**
  @brief Service callback for performing map evaluation
  @param req [ogm_msgs::ServerRequestEvaluation::Request&] The service request
  @param res [ogm_msgs::ServerRequestEvaluation::Response&] The service response
  @return bool
  **/
  bool MapEvaluator::evaluationCallback(ogm_msgs::ServerRequestEvaluation::Request& req,
    ogm_msgs::ServerRequestEvaluation::Response& res)
  {
    _groundTruthMap = mapToMat(req.groundTruthMap);
    _slamMap = mapToMat(req.slamMap);
    cv::imshow("groundTruthMap", _groundTruthMap );
    cv::waitKey(30);
    cv::imshow("slamMap", _slamMap);
    cv::waitKey(30);

    if (req.method == "OMSE")
    {

    }

    else if(req.method == "CMSE")
    {

    }

    else
    {
      ROS_ERROR("No such evaluation method");
      return false;
    }
    return true;
  }

  /**
  @brief Method that converts nav_msgs/OccupancyGrid to cv::Mat
  @oaram map [const nav_msgs::OccupancyGrid&] the OccupancyGrid map
  @return void
  **/
  cv::Mat MapEvaluator::mapToMat(const nav_msgs::OccupancyGrid& map)
  {
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    
    if (map.info.height*map.info.width != map.data.size())
    {
        ROS_ERROR("Data size doesn't match width*height: width = %d, height = %d, data size = %zu", map.info.width, map.info.height, map.data.size());
    }

    // transform the map in the same way the map_saver component does
    for (size_t i=0; i < map.info.height*map.info.width; i++)
    {
        if (map.data.at(i) == -1)
        {
            im.data[i] = 127;
        }
        else
        {
          im.data[i] = (100.0 - map.data.at(i))/100.0 *255.0;
        }
    }

    return im;
  }

  /**
  @brief Method that aligns the two maps using the transform received from server
  @return void
  **/
  void MapEvaluator::alignMaps()
  {
    
  }

}  // namespace ogm_evaluation
