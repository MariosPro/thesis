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

#include "ogm_evaluation/ogm_evaluation_descriptors/radius_statistics_descriptor.h"

namespace ogm_evaluation
{
    /**
    @brief Default Constructor
    @return void
    **/ 
    RadiusStatisticsDescriptor::RadiusStatisticsDescriptor():DescriptorExtractor()
    {

    }
    
    /**
    @brief function for description extract computation.
    @param image [const cv::Mat&] the image 
    @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
    @param descriptors [cv::Mat&] the descriptors to be extracted
    @return void
    **/
    void RadiusStatisticsDescriptor::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
    {
      int radius = 50;
      int obstacles;
      int frees;
      descriptors = cv::Mat(keypoints.size(), 2, CV_64FC1);

      for (int k = 0; k < keypoints.size(); k++)
      {
        obstacles;
        frees;
        cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
        cv::circle(mask, keypoints[k].pt, radius, cv::Scalar(255), -1);
        for(unsigned int i = 0; i < image.rows; i++)
          for(unsigned int j = 0; j < image.cols; j++)
            if( mask.at<unsigned char>(i, j) > 0)
            {
              //pixel (i,j) in original image is within that circle so do whatever you want.
              if(image.at<unsigned char>(i, j) == 0)
                obstacles++;
              else if(image.at<unsigned char>(i, j) == 255)
                frees++;
            }
        descriptors.at<double>(k, 0) = obstacles;
        descriptors.at<double>(k, 1) =  frees;
       }
    }
} // namespace ogm_evaluation
