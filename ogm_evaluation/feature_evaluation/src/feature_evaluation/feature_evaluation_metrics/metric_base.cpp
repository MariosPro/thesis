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

#include "feature_evaluation/feature_evaluation_metrics/metric_base.h"

namespace feature_evaluation
{
 /**
  @brief Default Constructor
  @param groundTruthMap [const cv::Mat& ] the ground truth map
  @param slamMap[const cv::Mat&] the slam produced Map
  @return void
  **/

  Metric::Metric(const cv::Mat& groundTruthMap,
                 const cv::Mat& slamMap)
  {
    _groundTruthMap = groundTruthMap;
    _slamMap = slamMap;
  }
  Metric::~Metric(void) 
  {
     //std::cout << "Destroying Metric Base" << std::endl;  
  };

  double Metric::getResult()
  {
    return _result;
  }

  int Metric::getMatches()
  {
    return _inliers;
  }

  double Metric::getAcceptance()
  {
    return _acceptance;
  }
 
  double Metric::getQuality()
  {
    return _quality;
  }
 
  double Metric::getOverlap()
  {
    return _overlap;
  }



} // namespace feature_evaluation
