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

#include "feature_evaluation/feature_evaluation_descriptors/annular_statistics_descriptor.h"

namespace feature_evaluation
{
    /**
    @brief Default Constructor
    @return void
    **/ 
    AnnularStatisticsDescriptor::AnnularStatisticsDescriptor():DescriptorExtractor()
    {
        //ROS_INFO_STREAM("Created AnnularStatisticsDescriptor instance");
    }
    
    /**
    @brief function for description extract computation.
    @param image [const cv::Mat&] the image 
    @param keypoints [std::vector<cv::Keypoint>&] the image detected keypoints
    @param descriptors [cv::Mat&] the descriptors to be extracted
    @return void
    **/
    void AnnularStatisticsDescriptor::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat* descriptors)
    {
      float previousRadius, initialRadius, radius, nextRadius;
      int obstacles;
      int frees;
      double min, max;
      float median, sum, count;
      float sse, dst;
      int numRings = 8;
      cv::Mat img, temp;
      std::vector<int> vals;
      cv::cvtColor(image, img, CV_GRAY2RGB);
      cv::Mat desc = cv::Mat(0, numRings * 5, CV_32FC1);
      std::vector<float> rowFeatures;
      float radiusRatio = 1.1;
      float totalLength = std::min(image.rows, image.cols)/5;
      float factor = 0;
      //std::cout << "FinalRadius=" <<  totalLength << std::endl;
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
          min = 255;
          max = median = sum = count = 0;
          cv::Scalar mean,std;
          cv::Rect r2 = cv::Rect(keypoints[k].pt.x -radius, keypoints[k].pt.y - radius, 2 * radius, 2 * radius);
          r2 = r2 & cv::Rect(0, 0, image.cols, image.rows);

          cv::Mat roi2;
          roi2 = image(r2);
          cv::Mat mask1 = cv::Mat::zeros(image.size(), image.type());
          cv::Mat mask2 = cv::Mat::zeros(image.size(), image.type());
          cv::circle(mask1, keypoints[k].pt, previousRadius, cv::Scalar::all(255), -1);
          cv::circle(mask2, keypoints[k].pt, radius, cv::Scalar::all(255), -1);
       /*   cv::Mat mask = cv::Mat::zeros(mask1.size(), mask1.type());*/

          //for (int i = 0; i < mask.rows; i++)
            //for(int j = 0; j < mask.cols; j++)
            //if(mask1.at<uchar>(i, j) == 0 && mask2.at<uchar>(i, j) == 255)
              /*mask.at<uchar>(i, j) = 255;*/
         cv::Mat mask;
         cv::subtract(mask2, mask1, mask);
         mask = mask(r2);
         cv::Mat circleCropped = mask & roi2;
    /*     cv::imshow("roi2", roi2);*/
         //cv::imshow("circleCropped", circleCropped);
         //cv::imshow("mask", mask);
         /*cv::waitKey(1000);*/
         vals.clear();
         for(unsigned int i = 0; i < circleCropped.rows; i++)
            for(unsigned int j = 0; j < circleCropped.cols; j++)
              if(mask.at<unsigned char>(i, j) > 0)
              {
                vals.push_back(circleCropped.at<uchar>(i,j));
       /*         if(min > circleCropped.at<uchar>(i,j))*/
                  //min = circleCropped.at<uchar>(i,j);
                //if(max < circleCropped.at<uchar>(i,j))
                  /*max = circleCropped.at<uchar>(i,j);*/
                count++;
               }

          cv::minMaxLoc(circleCropped, &min, &max, NULL, NULL, mask);
          if(count == 0)
            std::cout << "NO MASK" << std::endl;
          cv::meanStdDev(circleCropped, mean, std, mask);
          median = (float)getMedian(vals);
          rowFeatures.push_back((float)min);
          rowFeatures.push_back((float)max);
          rowFeatures.push_back(mean.val[0]);
          rowFeatures.push_back(std.val[0]);
          rowFeatures.push_back(median);
  /*      for (int i = 0; i < rowFeatures.size(); i++)*/
        //{
          //std::cout << rowFeatures[i] << " ";
        //}
        //std::cout << std::endl;

 /*         cv::circle(img, keypoints[k].pt, previousRadius, cv::Scalar(255, 0, 0), 1, 8);*/
          /*cv::circle(img, keypoints[k].pt, radius, cv::Scalar(255, 0, 0), 1, 8);*/
     /*     cv::imshow("annular",img);*/
          /*cv::waitKey(0);*/
          nextRadius = radiusRatio *(radius - previousRadius) + radius;
          previousRadius = radius;
          radius = nextRadius;
        }
        temp.release();
        temp = cv::Mat(1, numRings*5, CV_32FC1);
        memcpy(temp.data, rowFeatures.data(), rowFeatures.size()*sizeof(CV_32FC1));
        desc.push_back(temp);

/*        for (int i = 0; i < rowFeatures.size(); i++)*/
        //{
          //desc.at<float>(k, i) = rowFeatures[i];
        /*}*/
      }

      //std::cout << "FinalRadius=" <<  nextRadius << std::endl;
  /*    for (int i = 0; i < desc.rows; i++)*/
      //{
        //cv::normalize(desc.row(i), desc.row(i), 0, 1, cv::NORM_MINMAX, CV_32FC1);
        ////std::cout << desc.row(i) << " ";
      /*}*/
      //std::cout << std::endl;
 /*     cv::imshow("Annular Descriptors", img);*/
      /*cv::waitKey(1000);*/
      //ROS_INFO_STREAM("DESC=" <<desc.rows << " " << desc.cols);
      desc.copyTo(*descriptors);
    }
    double AnnularStatisticsDescriptor::getMedian(std::vector<int>& v)
    {
      if(v.empty()) return 0;
      else {
        std::sort(v.begin(), v.end());
      if(v.size() % 2 == 0)
        return (v[v.size()/2 - 1] + v[v.size()/2]) / 2;
      else
        return v[v.size()/2];
    }
     }
} // namespace feature_evaluation
