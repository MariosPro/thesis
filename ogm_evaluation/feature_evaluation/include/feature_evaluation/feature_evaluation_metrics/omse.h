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

#ifndef OMSE_H
#define OMSE_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "feature_evaluation/feature_evaluation_metrics/metric_base.h"
#include "feature_evaluation/feature_evaluation_utils/map_utils.h"
#include "feature_evaluation/feature_evaluation_utils/maps_alignment.h"

/**
@namespace feature_evaluation
@brief The main namespace for feature Evaluation
**/ 
namespace feature_evaluation
{
  /**
  @class OmseMetric
  @brief A class that provides Obstacle Mean square Error Metric implementation.
  Inherits publicly Metric
  **/ 
  class OmseMetric : public Metric {

    public:

      /**
      @brief Default Constructor
      @param groundTruthMap [const cv::Mat& ] the ground truth map
      @param slamMap[const cv::Mat&] the slam produced Map
      @param std::string [closestPointMethod] the closestPoint method to be used
      @param std::string [distNorm] the distance norm to be used
      @return void
      **/

      OmseMetric(const cv::Mat& groundTruthMap,
                 const cv::Mat& slamMap);

      /**
      @brief Default destructor
      @return void
      **/
      virtual ~OmseMetric(void) 
      {
      }

      /**
      @brief calculate the obstacle metric.
      @return void
      **/
      virtual void calculateMetric(Parameters params);

      /**
      @brief extracts the obstacle points from cv::Mat
      @param mapMat [const cv::Mat&] the map cv::Mat
      @return std::vector<cv::Point> the vector of obstacle points found in cv::Mat
      **/
      std::vector<cv::Point> extractObstaclePoints(const cv::Mat& mapMat);

      /**
      @brief Calculates the closest obstacle of a slamObstaclePoint from a groundTruthObstaclePoint (BruteForce)
      @param sp [cv::Point] an obstacle point of slam-produced map
      @param distNorm [std::string] the distance calculation norm (Manhattan/Euclidean)
      @return double the distance
      **/
      double bruteForceNearestNeighbor(cv::Point sp, std::string distNorm);

      /**
      @brief Calculates the distance of points given
      @param p1 [cv::Point] the first point
      @param p2 [cv::Point] the second point
      @param distNorm [std::string] the norm to be used (Manhattan/Euclidean)
      @return double the distance
      **/
      double calculateDistance(cv::Point p1, cv::Point p2, std::string distNorm);

      sensor_msgs::Image getMatchedImage();

      sensor_msgs::Image getMergedImage();


    private:

      //!< the vector of ground Truth map obstacle points
      std::vector<cv::Point> _groundTruthObstaclePoints;

      //!< the vector of slam-produced map obstacle points
      std::vector<cv::Point> _slamObstaclePoints;

      //!< the closestPoint method to be used
      std::string _closestPointMethod;

      //!< the distance norm to be used
      std::string _distNorm;

      //!< the brushFire array holds the min Manhattan distance from nearest
      //obstacle
      int **_brushfire;

      //!< the MapUtils instance
      MapUtils _mapUtils;
      
      //!< the Alignment instance
      Alignment _alignment;


  };
}
#endif
