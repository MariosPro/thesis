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

#ifndef CMSE_H
#define CMSE_H

/*#include <ros/ros.h>*/
/*#include <opencv2/opencv.hpp>*/
#include "ogm_evaluation/ogm_evaluation_metrics/metric_base.h"

/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  /**
  @class CmseMetric
  @brief A class that provides Corner Mean square Error Metric implementation.
  Inherits publicly Metric
  **/ 
  class CmseMetric : public Metric {

    public:

      /**
      @brief Default Constructor
      @param groundTruthMap [const cv::Mat& ] the ground truth map
      @param slamMap[const cv::Mat&] the slam produced Map
      @param int [method] the cmse method to be used
      @param int [distNorm] the distance norm to be used
      @return void
      **/

      CmseMetric(const cv::Mat& groundTruthMap,
                 const cv::Mat& slamMap,
                 int method,
                 int distNorm);

      /**
      @brief Default destructor
      @return void
      **/
      virtual ~CmseMetric(void) 
      {
      }

      /**
      @brief calculate the corner metric.
      @return void
      **/
      virtual void calculateMetric(void);

      /**
      @brief extracts the corners points from cv::Mat
      @param mapMat [const cv::Mat&] the map cv::Mat
      @return std::vector<cv::Point> the vector of  corner points found in cv::Mat
      **/
      std::vector<cv::Point> extractCorners(const cv::Mat& mapMat);

      /**
      @brief Calculates the closest corner point of a slamCorner from a groundTruthCorner (BruteForce)
      @param sp [cv::Point] a corner point of slam-produced map
      @param method [int] the distance calculation method (1-Manhattan 2-Euclidean)
      @return double the distance
      **/
      double bruteForceNearestNeighbor(cv::Point sp, int method);

      /**
      @brief Calculates the minimum distance of all free and unknown cells from the closest occupied cells
      @return void
      **/
      void brushfireSearch();

      /**
      @brief Calculates the distance of points given
      @param p1 [cv::Point] the first point
      @param p2 [cv::Point] the second point
      @param method [int] the norm to be used (1-Manhattan 2-Eucledian)
      @return double the distance
      **/
      double calculateDistance(cv::Point p1, cv::Point p2, int method);

    private:

      //!< the vector of ground Truth map corners points
      std::vector<cv::Point> _groundTruthCorners;

      //!< the vector of slam-produced map corners points
      std::vector<cv::Point> _slamCorners;

      //!< the cmse method to be used
      int _cmse_method;

      //!< the distance norm to be used
      int _distNorm;

      //!< the brushfire array holds the min Manhattan distance from nearest
      //corner
      int **_brushfire;
  };
}
#endif
