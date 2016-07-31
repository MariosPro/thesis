/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2016 OGM Validator
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
#ifndef DESCRIPTOR_EXTRACTOR_H
#define DESCRIPTOR_EXTRACTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace ogm_evaluation
{
  class DescriptorExtractor
  {
    public:
 
      /**
      @brief Virtual function for description extract computation. Implement this function
      on derived class to compute the individual descriptors of each detected keypoint.
      @return void
      **/ 
      virtual void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors) = 0;
 
      /**
      @brief Default Constructor
      @return void
      **/ 
      DescriptorExtractor(void)
      {
      };

      /**
      @brief Default destructor
      @return void
      **/ 
      virtual ~DescriptorExtractor(void)
      {
      }
  };
}
#endif
