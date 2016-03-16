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
#ifndef MAPS_ALIGNMENT_H
#define MAPS_ALIGNMENT_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ogm_msgs/MapPose.h"

/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  class Alignment
  {
    public:
      Alignment();

      void alignMaps(const ogm_msgs::MapPose& _transform,
                     cv::Mat& _groundTruthMap,
                     cv::Mat& _slamMap);
    private:
      //!<  the transform converting slamMap to groundTruthMap Coordinates System
      ogm_msgs::MapPose _transform; 
      
      cv::Mat _groundTruthMap;

      cv::Mat _slamMap;
  };
}
#endif
