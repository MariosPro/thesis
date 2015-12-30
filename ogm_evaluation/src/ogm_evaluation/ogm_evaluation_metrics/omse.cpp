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

#include "ogm_evaluation/ogm_evaluation_metrics/omse.h"

namespace ogm_evaluation
{
  /**
  @brief Default Constructor
  @param groundTruthMap [const cv::Mat& ] the ground truth map
  @param slamMap[const cv::Mat&] the slam produced Map
  @param int [method] the omse method to be used
  @param int [distNorm] the distance norm to be used
  @return void
  **/

  OmseMetric::OmseMetric(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap,
                         int method,
                         int distNorm)
                : Metric(groundTruthMap, slamMap)
  {
    _omse_method = method;
    _distNorm = distNorm;
  }

  /**
  @brief calculate the obstacle metric.
  @return void
  **/
  void OmseMetric::calculateMetric()
  {
     _result = 0;
    ROS_INFO_STREAM("GROUND POINTS= " << _groundTruthObstaclePoints.size());
    ROS_INFO_STREAM("SLAM POINTS= " << _slamObstaclePoints.size());

    _groundTruthObstaclePoints = extractObstaclePoints(_groundTruthMap);
    _slamObstaclePoints = extractObstaclePoints(_slamMap);
    ROS_INFO_STREAM("GROUND POINTS= " << _groundTruthObstaclePoints.size());
    ROS_INFO_STREAM("SLAM POINTS= " << _slamObstaclePoints.size());
    for (int i = 0; i < _slamObstaclePoints.size(); i++)
    {
      _result += bruteForceClosestPair(_slamObstaclePoints[i], _distNorm) *
                 bruteForceClosestPair(_slamObstaclePoints[i], _distNorm);
    }
    _result = _result / _slamObstaclePoints.size();
  }

  /**
  @brief extracts the obstacle points from cv::Mat
  @param mapMat [const cv::Mat&] the map cv::Mat
  @return std::vector<cv::Point> the vector of obstacle points found in cv::Mat
  **/
  std::vector<cv::Point> OmseMetric::extractObstaclePoints(const cv::Mat& mapMat)
  {
    std::vector<cv::Point> p;
    for (unsigned int i = 0; i < mapMat.rows; i++)
      for (unsigned int j = 0; j < mapMat.cols; j++)
      {
        if(mapMat.ptr<uchar>(i)[j] == 0)
          p.push_back(cv::Point(i, j));
/*        if(mapMat.at<uchar>(i, j) == 0)*/
          /*p.push_back(cv::Point(i,j));*/
      }
    return p;
  }

  /**
  @brief Calculates the closest obstacle of a slamObstaclePoint from a groundTruthObstaclePoint (BruteForce)
  @param sp [cv::Point] an obstacle point of slam-produced map
  @param method [int] the distance calculation method (1-Manhattan 2-Euclidean)
  @return double the distance
  **/
  double OmseMetric::bruteForceClosestPair(cv::Point sp, int method)
  {
    double minDst = calculateDistance(sp, _groundTruthObstaclePoints[0], method);
    double dst;
    for (int i = 0; i < _groundTruthObstaclePoints.size(); i++)
    {
      dst = calculateDistance(sp, _groundTruthObstaclePoints[i], method);
      if( dst < minDst )
        minDst = dst;
    }
    return minDst;
  }
  
  /**
  @brief Calculates the distance of points given
  @param p1 [cv::Point] the first point
  @param p2 [cv::Point] the second point
  @param method [int] the norm to be used (1-Manhattan 2-Euclidean)
  @return double the distance
  **/
  double OmseMetric::calculateDistance(cv::Point p1, cv::Point p2, int method)
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
