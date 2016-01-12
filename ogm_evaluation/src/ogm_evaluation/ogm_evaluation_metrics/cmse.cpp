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
******************************************************************************/

#include "ogm_evaluation/ogm_evaluation_metrics/cmse.h"

namespace ogm_evaluation
{
  /**
  @brief Default Constructor
  @param groundTruthMap [const cv::Mat& ] the ground truth map
  @param slamMap[const cv::Mat&] the slam produced Map
  @param int [method] the cmse method to be used
  @param int [distNorm] the distance norm to be used
  @return void
  **/

  CmseMetric::CmseMetric(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap,
                         int method,
                         int distNorm)
                : Metric(groundTruthMap, slamMap)
  {
    _cmse_method = method;
    _distNorm = distNorm;
  }

  /**
  @brief calculate the corner metric.
  @return void
  **/
  void CmseMetric::calculateMetric()
  {

  }

  /**
  @brief extracts the corners points from cv::Mat
  @param mapMat [const cv::Mat&] the map cv::Mat
  @return std::vector<cv::Point> the vector of  corner points found in cv::Mat
  **/
  std::vector<cv::Point> CmseMetric::extractCorners(const cv::Mat& mapMat)
  {
    std::vector<cv::Point> p;
    return p;
  }

  /**
  @brief Calculates the closest corner of a slamCornerPoint from a groundTruthCornerPoint (BruteForce)
  @param sp [cv::Point] a corner point of slam-produced map
  @param method [int] the distance calculation method (1-Manhattan 2-Euclidean)
  @return double the distance
  **/
  double CmseMetric::bruteForceNearestNeighbor(cv::Point sp, int method)
  {
    double minDst = calculateDistance(sp, _groundTruthCorners[0], method);
    double dst;
    for (int i = 0; i < _groundTruthCorners.size(); i++)
    {
      dst = calculateDistance(sp, _groundTruthCorners[i], method);
      if( dst < minDst )
        minDst = dst;
    }
    return minDst;
  }

  /**
  @brief Calculates the minimum distance of all free and unknown cells from the closest occupied cells
  @return void
  **/
  void CmseMetric::brushfireSearch()
  {
    _brushfire = new int*[_groundTruthMap.rows];
    for(int i = 0 ; i < _groundTruthMap.rows ; i++)
    {
      _brushfire[i] = new int[_groundTruthMap.cols];
      for(int j = 0 ; j < _groundTruthMap.cols; j++)
      {
        //TO DO: the wave can be spread into not corner points
        //initialize with 0 only corners Found in groundTruth
        if(_groundTruthMap.at<uchar>(i, j) >= 127) 
          _brushfire[i][j] = -1;
        else
          _brushfire[i][j] = 0;

      }
    }

    // Create brushfire (Manhattan distance transformation)
    bool foundWave = true;
    int currentWave = 0; //initial wave is Corners

    while(foundWave)
    {
      foundWave = false;
      for (int i = 0; i < _groundTruthMap.rows; i++)
        for(int j = 0; j < _groundTruthMap.cols; j++)
          if(_brushfire[i][j] == currentWave)
          {
            foundWave = true;
            if(i > 0 ) //This code checks the array bounds heading WEST
              if(_brushfire[i - 1][j] == -1) //This code checks the WEST direction
                _brushfire[i - 1][j] = currentWave + 1;

            if(i < (_groundTruthMap.rows -1)) //This code checks the array bounds heading EAST
              if(_brushfire[i + 1][j] == -1) //This code checks the EAST direction
                _brushfire[i + 1][j] = currentWave + 1;

            if(j > 0) //This code checks the array bounds heading SOUTH
              if(_brushfire[i][j - 1] == -1) //This code checks the SOUTH direction
                _brushfire[i][j - 1] = currentWave + 1;
            if(j < (_groundTruthMap.cols - 1)) //This code checks the array bounds heading NORTH
              if(_brushfire[i][j + 1] == -1) //This code checks the NORTH direction
                _brushfire[i][j + 1] = currentWave + 1;
          }
      currentWave++;
    }
  }

  /**
  @brief Calculates the distance of points given
  @param p1 [cv::Point] the first point
  @param p2 [cv::Point] the second point
  @param method [int] the norm to be used (1-Manhattan 2-Euclidean)
  @return double the distance
  **/
  double CmseMetric::calculateDistance(cv::Point p1, cv::Point p2, int method)
  {
    double dst;
    cv::Point diff = p1 - p2;
    if (method == 1)
      dst = std::abs(p1.x - p2.x) + std::abs(p1.y-p2.y);
    else if (method == 2)
      dst = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    else 
    {
      ROS_ERROR("No such method for distance calculation");
      return -1;
    }
    return dst;
  }
}  // namespace ogm_evaluation
