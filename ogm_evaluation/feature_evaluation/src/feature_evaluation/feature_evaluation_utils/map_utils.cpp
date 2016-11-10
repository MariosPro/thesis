/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2015 OGM Validator
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

#include "feature_evaluation/feature_evaluation_utils/map_utils.h"

namespace feature_evaluation
{
  /**
  @brief Default Constructor
  @return void
  **/
  MapUtils::MapUtils()
  {
  }
  
  /**
  @brief Calculates the minimum distance of all free and unknown cells from the closest occupied cells
  @param map [const cv::Mat&] the map cv::Mat
  @param _brushfire [** int] the array the distances to be saved
  @return void
  **/
  void MapUtils::brushfireSearch(const cv::Mat& map, int** brushfire)
  {
    for(int i = 0 ; i < map.rows ; i++)
    {
      for(int j = 0 ; j < map.cols; j++)
      {
        if(map.at<uchar>(i, j) >= 127) // the wave can be spread in free and unknown space
          brushfire[i][j] = -1;
        else
          brushfire[i][j] = 0;
      }
    }

    // Create brushfire (Manhattan distance transformation)
    bool foundWave = true;
    int currentWave = 0; //initial wave is Obstacles (0)

    while(foundWave)
    {
      foundWave = false;
      for (int i = 0; i < map.rows; i++)
        for(int j = 0; j < map.cols; j++)
          if(brushfire[i][j] == currentWave)
          {
            foundWave = true;
            if(i > 0 ) //This code checks the array bounds heading WEST
              if(brushfire[i - 1][j] == -1)//This code checks the WEST direction
                brushfire[i - 1][j] = currentWave + 1;

            if(i < (map.rows -1)) //This code checks the array bounds heading EAST
              if(brushfire[i + 1][j] == -1)//This code checks the EAST direction
                brushfire[i + 1][j] = currentWave + 1;

            if(j > 0) //This code checks the array bounds heading SOUTH
              if(brushfire[i][j - 1] == -1)//This code checks the SOUTH direction
                brushfire[i][j - 1] = currentWave + 1;
            if(j < (map.cols - 1)) //This code checks the array bounds heading NORTH
              if(brushfire[i][j + 1] == -1)//This code checks the NORTH direction
                brushfire[i][j + 1] = currentWave + 1;
          }
      currentWave++;
    }
  }

  /**
  @brief Calculates the average of the free cells on the brushfire array 
  @param _brushfire [** int] the array of the distances
  @return double the avg distance
  **/
  double MapUtils::meanBrushfireDistance(const cv::Mat& map, int** brushfire)
  {
    double sum = 0;
    int frees = 0;
    for (int i = 0; i < map.rows; i++)
      for (int j = 0;  j < map.cols; j++)
      {
        if(map.at<uchar>(i, j) != 127)
        {
          sum += brushfire[i][j];
          frees++;
        }
      }
    return sum/frees;
  }

  int MapUtils::maxBrushfireDistance(const cv::Mat& map, int** brushfire)
  {
    int max = 0;
    for (int i = 0; i < map.rows; i++)
      for (int j = 0;  j < map.cols; j++)
      {
        if(max < brushfire[i][j])
        {
          max = brushfire[i][j];
        }
      }
    return max;

  }
  /** decide whether point p is in the ROI.
  *** The ROI is a rotated rectangle whose 4 corners are stored in roi[] 
  **/
  bool MapUtils::isInROI(cv::Point p, std::vector<cv::Point2f> roi)
  {
      double pro[4];
      for(int i=0; i<4; ++i)
      {
          pro[i] = computeProduct(p, roi[i], roi[(i+1)%4]);
      }
      if(pro[0]*pro[2]<0 && pro[1]*pro[3]<0)
      {
          return true;
      }
      return false;
  }

  /** function pro = kx-y+j, take two points a and b,
  *** compute the line argument k and j, then return the pro value
  *** so that can be used to determine whether the point p is on the left or right
  *** of the line ab
  **/
  double MapUtils::computeProduct(cv::Point p, cv::Point2f a, cv::Point2f b)
  {
      double k = (a.y-b.y) / (a.x-b.x);
      double j = a.y - k*a.x;
      return k*p.x - p.y + j;
  }

  void MapUtils::findUsefulBoundaries(cv::Mat& map)
  {
    int largest_area=0;
    int largest_contour_index=0;
    cv::Rect bounding_rect;

    cv::Mat thr(map.rows, map.cols,CV_8UC1); 
    cv::Mat dst(map.rows, map.cols, CV_8UC1, cv::Scalar::all(0));
    cv::threshold(map, thr, 127, 255, CV_THRESH_TOZERO); //Threshold the unknown

    std::vector<std::vector<cv::Point> > contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;

      findContours(thr, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
     
       for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
        {
           double a= cv::contourArea( contours[i],false);  //  Find the area of contour
           if(a > largest_area)
           {
             largest_area = a;
             largest_contour_index = i;                //Store the index of largest contour
              bounding_rect = cv::boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
           }
        }

  /*  cv::Scalar color( 255,255,255);*/
    /*cv::drawContours( dst, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.*/

       int padding = 10;
       cv::Rect paddingRect = cv::Rect(bounding_rect.x - padding, bounding_rect.y - padding, bounding_rect.width + (padding * 2), bounding_rect.height + (padding * 2));

    cv::Rect mapRect = cv::Rect(0, 0, map.cols, map.rows);
    // rect that contains all that is in BOTH rects:
    cv::Rect fittingRect = paddingRect & mapRect;
    map = cv::Mat(map, fittingRect);
    /*imshow( "src", map );*/
    //imshow( "largest Contour", dst );
    /*cv::waitKey(0);*/
  }
  void MapUtils::morphologicalFiltering(cv::Mat& src)
  {
    // Create a structuring element (SE)
    int morph_size = 2;
    cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  
    cv::Mat thr;
    src.copyTo(thr);
      for (int i = 0; i < thr.rows; i++)
        for (int j = 0; j < thr.cols; j++)
          if(thr.at<uchar>(i, j) == 0)
            thr.at<uchar>(i, j) = 255;

     for (int i = 0; i < thr.rows; i++)
        for (int j = 0; j < thr.cols; j++)
          if(thr.at<uchar>(i, j) < 255)
            thr.at<uchar>(i, j) = 0;

     cv::Mat dst; // result matrix
     cv::morphologyEx( thr, dst, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 1 );   
    for (int i = 0; i < dst.rows; i++)
        for (int j = 0; j < dst.cols; j++)
          if(dst.at<uchar>(i, j) == 0)
            dst.at<uchar>(i, j) = 127;
  for (int i = 0; i < dst.rows; i++)
        for (int j = 0; j < dst.cols; j++)
          if(src.at<uchar>(i, j) == 127 && dst.at<uchar>(i,j)!=127)
            src.at<uchar>(i, j) = 255;

  /*  imshow("source", src);*/
    //imshow("result", dst);
    /*cv::waitKey(0);*/
  }
}  // namespace feature_evaluation
