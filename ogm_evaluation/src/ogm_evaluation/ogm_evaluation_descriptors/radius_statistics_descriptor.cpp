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
        ROS_INFO_STREAM("Created RadiusStatisticsDescriptor instance");
    }
    
    /**
    @brief function for description extract computation.
    @param image [const cv::Mat&] the image 
    @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
    @param descriptors [cv::Mat&] the descriptors to be extracted
    @return void
    **/
    void RadiusStatisticsDescriptor::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors)
    {
      ROS_INFO_STREAM("ENTER COMPUTE " << keypoints.size());
      int radius = 50;
      int obstacles;
      int frees;
      cv::Mat desc = cv::Mat(keypoints.size(), 2, CV_32FC1);

      for (int k = 0; k < keypoints.size(); k++)
      {
        obstacles = 0;
        frees = 0;
        cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
        cv::circle(mask, keypoints[k].pt, radius, cv::Scalar(255), -1);
        for(unsigned int i = 0; i < image.rows; i++)
          for(unsigned int j = 0; j < image.cols; j++)
            if(mask.at<unsigned char>(i, j) > 0)
            {
              //pixel (i,j) in original image is within that circle so do whatever you want.
              if(image.at<unsigned char>(i, j) == 0)
                obstacles++;
              else if(image.at<unsigned char>(i, j) == 255)
                frees++;
            }
        desc.at<float>(k, 0) = obstacles;
        desc.at<float>(k, 1) =  frees;
      }
      desc.copyTo(*descriptors);
    }
} // namespace ogm_evaluation
