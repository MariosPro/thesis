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

#include "feature_evaluation/feature_evaluation_descriptors/circle_intersection_descriptor.h"

namespace feature_evaluation
{
    /**
    @brief Default Constructor
    @return void
    **/ 
    CircleIntersectionDescriptor::CircleIntersectionDescriptor():DescriptorExtractor()
    {
        //ROS_INFO_STREAM("Created CircleIntersectionDescriptor instance");
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
      int intersections, crosses;
      int numRings = 8;
      float previousRadius, initialRadius, radius, nextRadius;
      float x, y;
      cv::Mat img;
      cv::cvtColor(image, img, CV_GRAY2RGB);
      cv::Mat desc = cv::Mat(0, numRings, CV_32FC1);
      cv::Mat temp;       
      std::vector<float> rowFeatures;
      float radiusRatio = 1.1;
      float totalLength = std::min(image.rows, image.cols)/5;
      float factor = 0;
      for (int i = 0; i < numRings; i++)
      {
         factor += std::pow(radiusRatio, i);
      }
      
      initialRadius = totalLength  / factor;

      for (int k = 0; k < keypoints.size(); k++)
      {
        previousRadius = 0;
        radius = initialRadius;
        nextRadius = 0;
        rowFeatures.clear();
        for (int l = 0; l < numRings; l++)
        {
          radius+= 25;
          intersections = 0;
          crosses = 0;
          for (int theta = 0; theta < 360; theta++)
          {
            x = (int) round((radius * cos(theta * M_PI / 180)) + keypoints[k].pt.x);
            y = (int) round((radius * sin(theta * M_PI / 180)) + keypoints[k].pt.y);

            // check if circle points are inside image boundaries
            if(x >= 0 && x < image.cols && y >= 0 && y < image.rows )
            {
              if( (image.at<unsigned char>(y, x) == 0 && crosses == 0) || image.at<unsigned char>(y, x) == 127) 
              {
                 crosses ++;
              }
              if(image.at<unsigned char>(y, x) == 255 && crosses == 1)
              {
                intersections ++;
                crosses = 0;
              }
            }
          }
          rowFeatures.push_back((float) intersections);
          temp.release();
          temp = cv::Mat(1, numRings, CV_32FC1);
          memcpy(temp.data, rowFeatures.data(), rowFeatures.size()*sizeof(CV_32FC1));
          //cv::circle(img, keypoints[k].pt, radius, cv::Scalar(255, 0, 0), 1, 8);
          nextRadius = radiusRatio *(radius - previousRadius) + radius;
          previousRadius = radius;
          radius = nextRadius;
        }
        desc.push_back(temp);

       /* if(k == 100)*/
        /*{*/
      /*    for (int i = 0; i < rowFeatures.size(); i++)*/
          //{
            //desc.push_back(rowFeatures[i]);
            ////std::cout << rowFeatures[i] << " ";
          /*}*/
        //}
      }
/*      cv::imshow("Radius Descriptors", img);*/
      /*cv::waitKey(1000);*/
      //ROS_INFO_STREAM("DESC=" <<desc.rows << " " << desc.cols);
      desc.copyTo(*descriptors);
    }
} // namespace feature_evaluation
