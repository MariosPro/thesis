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

    brushfireSearch();
    for (int i = 0; i < _slamObstaclePoints.size(); i++)
    {
      if(_omse_method == 0)
      {
        _result += bruteForceNearestNeighbor(_slamObstaclePoints[i], _distNorm) *
                 bruteForceNearestNeighbor(_slamObstaclePoints[i], _distNorm);
      }
      else
      {
        _result += _brushfire[_slamObstaclePoints[i].x][_slamObstaclePoints[i].y] *
                 _brushfire[_slamObstaclePoints[i].x][_slamObstaclePoints[i].y];
      }
    }
    _result = _result / _slamObstaclePoints.size();

    for (int i = 0; i < _groundTruthMap.rows; i++)
      delete[] _brushfire[i];
    delete[] _brushfire;
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
  double OmseMetric::bruteForceNearestNeighbor(cv::Point sp, int method)
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
  @brief Calculates the minimum distance of all free and unknown cells from the closest occupied cells
  @return void
  **/
  void OmseMetric::brushfireSearch()
  {
    _brushfire = new int*[_groundTruthMap.rows];
    for(int i = 0 ; i < _groundTruthMap.rows ; i++)
    {
      _brushfire[i] = new int[_groundTruthMap.cols];
      for(int j = 0 ; j < _groundTruthMap.cols; j++)
      {
        if(_groundTruthMap.at<uchar>(i, j) >= 127) // the wave can be spread in free and unknown space
          _brushfire[i][j] = -1;
        else
          _brushfire[i][j] = 0;

      }
    }

    // Create brushfire (Manhattan distance transformation)
    bool foundWave = true;
    int currentWave = 0; //initial wave is Obstacles (0)

    while(foundWave)
    {
      foundWave = false;
      for (int i = 0; i < _groundTruthMap.rows; i++)
        for(int j = 0; j < _groundTruthMap.cols; j++)
          if(_brushfire[i][j] == currentWave)
          {
            foundWave = true;
            if(i > 0 ) //This code checks the array bounds heading WEST
              if(_brushfire[i - 1][j] == -1)//This code checks the WEST direction
                _brushfire[i - 1][j] = currentWave + 1;

            if(i < (_groundTruthMap.rows -1)) //This code checks the array bounds heading EAST
              if(_brushfire[i + 1][j] == -1)//This code checks the EAST direction
                _brushfire[i + 1][j] = currentWave + 1;

            if(j > 0) //This code checks the array bounds heading SOUTH
              if(_brushfire[i][j - 1] == -1)//This code checks the SOUTH direction
                _brushfire[i][j - 1] = currentWave + 1;
            if(j < (_groundTruthMap.cols - 1)) //This code checks the array bounds heading NORTH
              if(_brushfire[i][j + 1] == -1)//This code checks the NORTH direction
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
