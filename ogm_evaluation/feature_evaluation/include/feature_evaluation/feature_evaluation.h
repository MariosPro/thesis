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
#ifndef FEATURE_EVALUATION_H
#define FEATURE_EVALUATION_H

#include <chrono>
#include <vector>
#include <string>
#include <math.h> 
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ogm_communications/ServerRequestEvaluation.h"
#include "ogm_communications/MapPose.h"
#include "feature_evaluation/feature_evaluation_metrics/parameters.h"
#include "feature_evaluation/feature_evaluation_metrics/metric_base.h"
#include "feature_evaluation/feature_evaluation_utils/maps_alignment.h"
#include "feature_evaluation/feature_evaluation_utils/map_utils.h"
#include "feature_evaluation/feature_metric_factory.h"
/**
@namespace feature_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace feature_evaluation
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
      @param req [ogm_communications::ServerRequestEvaluation::Request&] The service request
      @param res [ogm_communications::ServerRequestEvaluation::Response&] The service response
      @return bool
      **/
      bool evaluationCallback(ogm_communications::ServerRequestEvaluation::Request& req,
        ogm_communications::ServerRequestEvaluation::Response& res);

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
        ogm_communications::MapPose _transform; 

        //!< The Metrics Factory
        MetricFactory _metric_factory;

        //!< Pointer to the Metric used

        boost::shared_ptr<Metric> _metric;

        //!< The maps Alignment instance
        Alignment _alignment;

        //!< the Parameters instance
        Parameters _params;

        MapUtils* _mapUtils;
  };
}
#endif
