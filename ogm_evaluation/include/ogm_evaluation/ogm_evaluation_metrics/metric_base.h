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

#ifndef METRIC_H
#define METRIC_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "ogm_evaluation/ogm_evaluation_metrics/parameters.h"

/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  /**
  @class Metric
  @brief A class that provides metric abstraction
  **/ 
  class Metric 
  {
    public:

      /**
      @brief Virtual function for metric calculation. Implement this function
      on derived class to calculate the individual metrics.
      @return void
      **/ 
      virtual void calculateMetric(Parameters Params) = 0;

      /**
      @brief Default destructor
      @return void
      **/ 
      virtual ~Metric(void) 
      {
      }
      
      double getResult();

      virtual sensor_msgs::Image getMatchedImage() = 0;

      virtual sensor_msgs::Image getMergedImage() = 0;

    protected:

      /**
      @brief Default Constructor
      @param groundTruthMap [const cv::Mat& ] the ground truth map
      @param slamMap[const cv::Mat&] the slam produced Map
      @return void
      **/
      Metric(const cv::Mat& groundTruthMap,
             const cv::Mat& slamMap);

    protected:

      //!< the ground Truth Map
      cv::Mat _groundTruthMap;

      //!< the slam produced Map
      cv::Mat _slamMap;

      //!< the metric result
      double _result;

  };
}
#endif
