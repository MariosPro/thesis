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
#ifndef OGM_EVALUATION_H
#define OGM_EVALUATION_H

#include <vector>
#include <string>
#include <math.h> 
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ogm_msgs/ServerRequestEvaluation.h"
#include "ogm_msgs/MapPose.h"
#include "ogm_evaluation/ogm_evaluation_metrics/parameters.h"
#include "ogm_evaluation/ogm_evaluation_metrics/metric_base.h"
#include "ogm_evaluation/ogm_evaluation_utils/maps_alignment.h"
#include "ogm_evaluation/ogm_metric_factory.h"
/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  class MapEvaluator
  {
    public:

      /**
      @brief Default constructor
      @param argc [int] Number of input arguments
      @param argv [char**] The input arguments
      @return void
      **/
      MapEvaluator(int argc, char** argv);

      //!< Services --------------------------

      /**
      @brief Service callback for performing map evaluation
      @param req [ogm_msgs::ServerRequestEvaluation::Request&] The service request
      @param res [ogm_msgs::ServerRequestEvaluation::Response&] The service response
      @return bool
      **/
      bool evaluationCallback(ogm_msgs::ServerRequestEvaluation::Request& req,
        ogm_msgs::ServerRequestEvaluation::Response& res);

    private:

      /**
      @brief Method that converts nav_msgs/OccupancyGrid to cv::Mat
      @oaram map [const nav_msgs::OccupancyGrid&] the OccupancyGrid map
      @return void
      **/
      cv::Mat mapToMat(const nav_msgs::OccupancyGrid& map);

    private:

        //!< The Ros node Handle
        ros::NodeHandle _nh;

        //!< Service server for performing map evaluation
        ros::ServiceServer _mapEvaluationService;

        //!< the ground Truth map
        cv::Mat _groundTruthMap;

        //!< the slam produced map
        cv::Mat _slamMap;

        cv::Mat diff;

        //!<  the transform converting slamMap to groundTruthMap Coordinates System
        ogm_msgs::MapPose _transform; 

        //!< The Metrics Factory
        MetricFactory _metric_factory;

        //!< Pointer to the Metric used
        Metric* _metric;

        //!< The maps Alignment instance
        Alignment _alignment;

        //!< the Parameters instance
        Parameters _params;
  };
}
#endif
