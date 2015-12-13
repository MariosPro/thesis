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
    if (req.method == "OMSE")
    {

    }

    else if(req.method == "CMSE")
    {

    }

    else
    {
      ROS_ERROR("No such evaluation method");
    }
  }

  /**
  @brief Method that converts nav_msgs/OccupancyGrid to cv::Mat
  @oaram map [const nav_msgs::OccupancyGrid&] the OccupancyGrid map
  @return void
  **/
  void MapEvaluator::convertOccupancyGridToCvMat(const nav_msgs::OccupancyGrid& map)
  {
    
  }
}  // namespace ogm_evaluation
