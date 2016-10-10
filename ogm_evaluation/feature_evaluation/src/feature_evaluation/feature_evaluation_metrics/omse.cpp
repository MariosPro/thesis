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

#include "feature_evaluation/feature_evaluation_metrics/omse.h"

namespace feature_evaluation
{
  /**
  @brief Default Constructor
  @param groundTruthMap [const cv::Mat& ] the ground truth map
  @param slamMap[const cv::Mat&] the slam produced Map
  @param std::string [closestPointMethod] the closestPoint method to be used
  @param std::string [distNorm] the distance norm to be used
  @return void
  **/

  OmseMetric::OmseMetric(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap): Metric(groundTruthMap, slamMap)
  {
  }

  /**
  @brief calculate the obstacle metric.
  @return void
  **/
  void OmseMetric::calculateMetric(Parameters params)
  {
     _result = 0;
   /*  cv::Mat map1 = cv::Mat::zeros(10,10, CV_8UC1);*/
     //cv::Mat map2 = cv::Mat::zeros(10,10, CV_8UC1);
     //for(int i =0; i < map1.rows; i++)
       //for(int j = 0; j < map1.cols; j++)
         //if((i < 4 || j < 4) || (i > 7 || j > 7))
         //{
           //map1.at<uchar>(i,j)=255;
           //map2.at<uchar>(i,j)=255;
         /*}*/
     //_alignment.ICP(map1, map2, 5);
     //_alignment.ICP(_groundTruthMap, _slamMap, 5);
     std::vector<cv::Point> map1Points, map2Points;

    map1Points = extractObstaclePoints(_groundTruthMap);
    map2Points = extractObstaclePoints(_slamMap);
 
    if(map1Points.size() == 0 || map2Points.size() == 0)
    {
      ROS_ERROR("No Obstacle Points detected in one or neither maps");
      //exit(0);
    }

    cv::Mat map1, map2;
   
    if(map1Points.size() > map2Points.size())
    {
      _groundTruthMap.copyTo(map1);
      _slamMap.copyTo(map2);
      _groundTruthObstaclePoints.insert(_groundTruthObstaclePoints.end(), map1Points.begin(), map1Points.end());
      _slamObstaclePoints.insert(_slamObstaclePoints.end(), map2Points.begin(), map2Points.end());
    }
    else
    {
      _groundTruthMap.copyTo(map2);
      _slamMap.copyTo(map1);
    _groundTruthObstaclePoints.insert(_groundTruthObstaclePoints.end(), map2Points.begin(), map2Points.end());
      _slamObstaclePoints.insert(_slamObstaclePoints.end(), map1Points.begin(), map1Points.end());
    }
 
    ROS_INFO_STREAM("GROUND POINTS= " << _groundTruthObstaclePoints.size());
    ROS_INFO_STREAM("SLAM POINTS= " << _slamObstaclePoints.size());

    if(params.closestPointMethod == "Brushfire")
    {
      _brushfire = new int*[map1.rows];
      for(int i = 0; i < map1.rows; i++)
        _brushfire[i] = new int[map1.cols];
      _mapUtils.brushfireSearch(map1, _brushfire);
      double dist = _mapUtils.meanBrushfireDistance(map1, _brushfire);
      ROS_INFO("ERE");
    }

    for (int i = 0; i < _slamObstaclePoints.size(); i++)
    {
      if(params.closestPointMethod == "NearestNeighbor")
      {
        double dist = bruteForceNearestNeighbor(_slamObstaclePoints[i], params.distNorm);
        _result += dist * dist;
      }
      else if(params.closestPointMethod == "Brushfire")
      {
        _result += _brushfire[_slamObstaclePoints[i].x][_slamObstaclePoints[i].y] *
                 _brushfire[_slamObstaclePoints[i].x][_slamObstaclePoints[i].y];
      }
      else
      {
        ROS_ERROR("[OMSE] NO SUCH CLOSEST POINT METHOD");
      }
    }
    _result = _result / _slamObstaclePoints.size();
 
    if(params.closestPointMethod == "Brushfire")
    {
      for (int i = 0; i < map1.rows; i++)
        delete[] _brushfire[i];
      delete[] _brushfire;
    }
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
      }
    return p;
  }

  /**
  @brief Calculates the closest obstacle of a slamObstaclePoint from a groundTruthObstaclePoint (BruteForce)
  @param sp [cv::Point] an obstacle point of slam-produced map
  @param distNorm [std::string] the distance calculation norm (1-Manhattan 2-Euclidean)
  @return double the distance
  **/
  double OmseMetric::bruteForceNearestNeighbor(cv::Point sp, std::string distNorm)
  {
    double minDst = calculateDistance(sp, _groundTruthObstaclePoints[0], distNorm);
    double dst;
    for (int i = 0; i < _groundTruthObstaclePoints.size(); i++)
    {
      dst = calculateDistance(sp, _groundTruthObstaclePoints[i], distNorm);
      if( dst < minDst )
        minDst = dst;
    }
    return minDst;
  }

  /**
  @brief Calculates the distance of points given
  @param p1 [cv::Point] the first point
  @param p2 [cv::Point] the second point
  @param distNorm [std::string] the norm to be used (Manhattan/Euclidean)
  @return double the distance
  **/
  double OmseMetric::calculateDistance(cv::Point p1, cv::Point p2, std::string distNorm)
  {
    double dst;
    cv::Point diff = p1 - p2;
    if ( distNorm == "Manhattan")
      dst = std::abs(p1.x - p2.x) + std::abs(p1.y-p2.y);
    else if (distNorm == "Euclidean")
      dst = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    else 
    {
      ROS_ERROR("No such method for distance calculation");
      return -1;
    }
    return dst;
  }

  sensor_msgs::Image OmseMetric::getMatchedImage()
  {

  }

  sensor_msgs::Image OmseMetric::getMergedImage()
  {

  }

}  // namespace feature_evaluation
