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

#include "ogm_evaluation/ogm_evaluation_descriptors/mean_rays_descriptor.h"

namespace ogm_evaluation
{
    /**
    @brief Default Constructor
    @return void
    **/ 
    MeanRaysDescriptor::MeanRaysDescriptor():DescriptorExtractor()
    {
        ROS_INFO_STREAM("Created MeanRaysDescriptor instance");
    }
    
    /**
    @brief function for description extract computation.
    @param image [const cv::Mat&] the image 
    @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
    @param descriptors [cv::Mat&] the descriptors to be extracted
    @return void
    **/
    void MeanRaysDescriptor::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors)
    {
      int radius;
      float x, y;
      cv::Mat img;
      cv::cvtColor(image, img, CV_GRAY2RGB);
      cv::Mat desc = cv::Mat(keypoints.size(), 1, CV_32FC1);
      std::vector<float> rowFeatures;
      int sumOfRays, thetaIncrement;
      bool outOfBounds;
      std::vector<cv::Point> p;
      std::vector<int> rays;
      thetaIncrement = 1;
      for (int k = 0; k < keypoints.size(); k++)
      {
        sumOfRays = 0;
        p.clear();
        rays.clear();
        
        for (int theta = 0; theta < 360; theta = theta + thetaIncrement)
        {
          outOfBounds = false;
          //ROS_INFO_STREAM("THETA=" << theta);
          radius = 0;
          for(;;)
          {
            radius += 1;
            x = (int) round((radius * cos(theta * M_PI / 180)) + keypoints[k].pt.x);
            y = (int) round((radius * sin(theta * M_PI / 180)) + keypoints[k].pt.y);
            if(x < 0)
            {
              x = 0;
              outOfBounds = true;
            }

            if(y < 0)
            {
              y = 0;
              outOfBounds = true;
            }
            
            if(y > image.rows - 1)
            {
              y = image.rows -1;
              outOfBounds = true;
            }
            
            if(x > image.cols -1)
            {
              x = image.cols - 1;
              outOfBounds = true;
            }
            
            if(image.at<unsigned char>(y, x) == 0 || outOfBounds)
            {
              sumOfRays += radius;
              p.push_back(cv::Point(x, y));
              rays.push_back(radius);
              break;
            }
          }
        }
        desc.at<float>(k, 1) = (float)sumOfRays / (360 / thetaIncrement);
        //if(k == 6){
        //ROS_INFO_STREAM("POINTS SIZE="<< p.size());
        for (int i = 0; i < p.size(); i++)
        {
          cv::line(img, keypoints[k].pt, p[i], cv::Scalar(255, 0, 0), 2, 8);
      /*    std::cout << p[i] << " ";*/
          /*std::cout << rays[i] << " ";*/
        }
        //std::cout << std::endl;
         //}
      }
      ROS_INFO_STREAM("DESC=" << desc.rows << " " << desc.cols);
      std::cout << "Desc = "<< std::endl << " "  << desc << std::endl << std::endl;
      desc.copyTo(*descriptors);
      cv::imshow("MeanRays Descriptors", img);
      cv::waitKey(0);

    }
} // namespace ogm_evaluation
