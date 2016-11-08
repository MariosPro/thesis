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

#ifndef ANNULAR_STATISTICS_DESCRIPTOR_H
#define ANNULAR_STATISTICS_DESCRIPTOR_H

#include "feature_evaluation/feature_evaluation_descriptors/descriptor_extractor.h"

namespace feature_evaluation
{

  class AnnularStatisticsDescriptor : public DescriptorExtractor
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
      AnnularStatisticsDescriptor(void);
 
      /**
      @brief Default destructor
      @return void
      **/ 
      virtual ~AnnularStatisticsDescriptor(void)
      {
      };

      double getMedian(std::vector<int>& v);

  };
}
#endif

