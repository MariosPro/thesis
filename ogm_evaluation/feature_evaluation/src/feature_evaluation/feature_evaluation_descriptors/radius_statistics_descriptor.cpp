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

#include "feature_evaluation/feature_evaluation_descriptors/radius_statistics_descriptor.h"

namespace feature_evaluation
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
      int radius;
      int obstacles;
      int frees;
      float sse, dst;
      cv::Mat img;
      cv::cvtColor(image, img, CV_GRAY2RGB);
      //image.copyTo(img);
      cv::Mat desc = cv::Mat(keypoints.size(), 48, CV_32FC1);
      std::vector<float> rowFeatures;
      int radiusIncrement = std::min(image.rows, image.cols)/20;
      std::cout<< "radiusIncrement" << radiusIncrement << std::endl;
      std::cout << keypoints.size();
      for (int k = 0; k < keypoints.size(); k++)
      {
        //std::cout << k << std::endl;
        radius = 0;
        rowFeatures.clear();
        for (int l = 0; l < 6; l++)
        {
          radius+= radiusIncrement;
                 //get the Rect containing the circle:
    /*      cv::Rect r(keypoints[k].pt.x-radius, keypoints[k].pt.y-radius, radius*2,radius*2);*/
          //r = r & cv::Rect(0, 0, image.cols, image.rows);
          //// obtain the image ROI:
          //cv::Mat roi(image, r);
  
          
          //// make a black mask same size:
          /*cv::Mat mask = cv::Mat::zeros(roi.size(), roi.type());*/
          
          //with a white, filled circle in it:
          
          for (int startAngle = 0; startAngle < 360; startAngle = startAngle + 90)
          {
            obstacles = 0;
            frees = 0;
            sse = 0;
            //get the Rect containing the circle:
            cv::Point topleft;
            switch(startAngle)
            {
              case 0 : topleft.x = keypoints[k].pt.x;
                       topleft.y = keypoints[k].pt.y-radius;
                       break;
              case 90 : topleft.x = keypoints[k].pt.x-radius; 
                        topleft.y = keypoints[k].pt.y-radius;
                       break;
              case 180 : topleft.x = keypoints[k].pt.x-radius;
                         topleft.y = keypoints[k].pt.y;
                       break;
              case 270 : topleft.x = keypoints[k].pt.x;
                         topleft.y = keypoints[k].pt.y;
                       break;
            }
            cv::Point br; //= cv::Point(topleft.x + radius, topleft.y + radius);
            br.x = topleft.x + radius;
            br.y = topleft.y + radius;
            cv::Rect r(topleft, br);
            r = r & cv::Rect(0, 0, image.cols, image.rows);
            //std::cout << r.width<< " " << r.height << std::endl;
            // obtain the image ROI:
            cv::Mat roi(image, r);
    
            
            // make a black mask same size:
            cv::Mat mask = cv::Mat::zeros(roi.size(), roi.type());

            cv::ellipse(mask ,cv::Point(radius, radius), cv::Size(radius,radius),0, startAngle, startAngle + 90, cv::Scalar::all(255), -1);
            //cv::circle(mask, cv::Point(radius,radius), radius, cv::Scalar::all(255), -1);
            //cv::circle(mask, keypoints[k].pt, radius, cv::Scalar(255), -1);
            //cv::Mat imagePart = cv::Mat::zeros(image.size(), image.type());
            //image.copyTo(imagePart, mask);
            
            cv::Mat circleCropped = roi & mask;
            if(circleCropped.rows == 0 || circleCropped.cols == 0)
              ROS_INFO("DFDF");
     /*       std::cout << "keypoint=" << keypoints[k].pt << std::endl;*/
            //std::cout << "circleCropped=" << circleCropped.size() << std::endl;


   /*        cv::imshow("circleCropped", circleCropped);*/
           //cv::imshow("mask", mask);
           /*cv::waitKey(1000);*/
            for(unsigned int i = 0; i < circleCropped.rows; i++)
              for(unsigned int j = 0; j < circleCropped.cols; j++)
                if(mask.at<unsigned char>(i, j) > 0)
                {
                  //pixel (i,j) in original circleCropped is within that circle so do whatever you want.
                  if(circleCropped.at<unsigned char>(i, j) == 0)
                  {
                    obstacles++;
                    //dst = cv::sqrt(pow((i-keypoints[k].pt.x), 2) + pow((j-keypoints[k].pt.y), 2));
                    //sse += dst*dst;
                  }
                  else if(circleCropped.at<unsigned char>(i, j) == 255)
                    frees++;
                }
            //sse = sse / obstacles;
            float obstaclesRatio = obstacles+frees != 0 ? (float)obstacles / (obstacles + frees) : 0;
            float freesRatio = obstacles+frees != 0 ? (float)frees / (obstacles + frees) : 0;
            rowFeatures.push_back(obstaclesRatio);
            rowFeatures.push_back(freesRatio);
            //rowFeatures.push_back(sse);
            cv::ellipse(img ,keypoints[k].pt, cv::Size(radius,radius), 0, startAngle, startAngle + 90, cv::Scalar(255, 0, 0), 1, 8);
            //cv::circle(img, keypoints[k].pt, radius, cv::Scalar(255, 0, 0), 1, 8);
          }
        }
        //std::cout << rowFeatures.size() << std::endl;
        for (int i = 0; i < rowFeatures.size(); i++)
        {
          desc.at<float>(k, i) = rowFeatures[i];
          std::cout << rowFeatures[i] << " ";
        }
        std::cout << std::endl;
      }
      std::cout <<"wtf" << std::endl;
      cv::imshow("Radius Descriptors", img);
      cv::waitKey(1000);
      //ROS_INFO_STREAM("DESC=" <<desc.rows << " " << desc.cols);
      desc.copyTo(*descriptors);
    }
} // namespace feature_evaluation
