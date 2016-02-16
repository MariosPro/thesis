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

#include "ogm_evaluation/ogm_evaluation_descriptors/circle_intersection_descriptor.h"

namespace ogm_evaluation
{
    /**
    @brief Default Constructor
    @return void
    **/ 
    CircleIntersectionDescriptor::CircleIntersectionDescriptor():DescriptorExtractor()
    {
        ROS_INFO_STREAM("Created CircleIntersectionDescriptor instance");
    }
    
    /**
    @brief function for description extract computation.
    @param image [const cv::Mat&] the image 
    @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
    @param descriptors [cv::Mat&] the descriptors to be extracted
    @return void
    **/
    void CircleIntersectionDescriptor::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors)
    {
      ROS_INFO_STREAM("ENTER COMPUTE " << keypoints.size());
      int radius, intersections, crosses;
      float x, y;
      cv::Mat img, img1;
      //image.convertTo(img1, CV_32FC1);
      cv::cvtColor(image, img, CV_GRAY2RGB);
      //image.copyTo(img);
      cv::Mat desc = cv::Mat(keypoints.size(), 6, CV_32FC1);
      std::vector<float> rowFeatures;
      for (int k = 0; k < keypoints.size(); k++)
      {
        radius = 0;
        rowFeatures.clear();
        for (int l = 0; l < 6; l++)
        {
          radius+= 25;
          intersections = 0;
          cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
          cv::circle(mask, keypoints[k].pt, radius, cv::Scalar(255), 1);
       /*  cv::imshow("mask", mask);*/
         /*cv::waitKey(0);*/
          crosses = 0;
          for (int theta = 0; theta < 360; theta++)
          {
            x = (int) round((radius * cos(theta * M_PI / 180)) + keypoints[k].pt.x);
            y = (int) round((radius * sin(theta * M_PI / 180)) + keypoints[k].pt.y);

            // check if circle points are inside image boundaries
            if(x >= 0 && x < image.rows && y >= 0 && y < image.cols )
            {
              /*if(theta == 0)*/
              /*{*/
            /*  ROS_INFO_STREAM("CIRCLE POINT X,Y=" << x <<" " << y);*/
              /*ROS_INFO_STREAM("IMAGE=" << (int) image.at<unsigned char>(x,y));*/
              //}
              if(image.at<unsigned char>(x, y) == 0 && crosses == 0)
              {
                 crosses += 1;
                 //ROS_INFO_STREAM("ENTER OBSTACLE AREA crosses=" << crosses);
              }
              if(image.at<unsigned char>(x, y) == 255 && crosses == 1)
              {
                intersections += 1;
                crosses = 0;
                //ROS_INFO_STREAM("EXIT OBSTACLE AREA crosses=" << crosses);
              }
            }
          }
          rowFeatures.push_back((float) intersections);
          cv::circle(img, keypoints[1].pt, radius, cv::Scalar(255, 0, 0), 2, 8);
        }
        if(k==1)
        for (int i = 0; i < rowFeatures.size(); i++)
        {
          desc.at<float>(k, i) = rowFeatures[i];
          std::cout << rowFeatures[i] << " ";
        }
        //std::cout << std::endl;
      }
      cv::imshow("Radius Descriptors", img);
      cv::waitKey(1000);
      ROS_INFO_STREAM("DESC=" <<desc.rows << " " << desc.cols);
      desc.copyTo(*descriptors);
    }
} // namespace ogm_evaluation
