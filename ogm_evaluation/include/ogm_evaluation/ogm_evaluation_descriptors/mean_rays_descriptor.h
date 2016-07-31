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

#ifndef MEAN_RAYS_DESCRIPTOR_H
#define MEAN_RAYS_DESCRIPTOR_H

#include <math.h>
#include "ogm_evaluation/ogm_evaluation_descriptors/descriptor_extractor.h"

namespace ogm_evaluation
{

  class MeanRaysDescriptor : public DescriptorExtractor
  {
    public:
 
      /**
      @brief function for description extract computation.
      @param image [const cv::Mat&] the image 
      @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
      @param descriptors [cv::Mat&] the descriptors to be extracted
      @return void
      **/
      virtual void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors);

      /**
      @brief Default Constructor
      @return void
      **/ 
      MeanRaysDescriptor(void);
 
      /**
      @brief Default destructor
      @return void
      **/ 
      virtual ~MeanRaysDescriptor(void)
      {
      };
  };
}
#endif

