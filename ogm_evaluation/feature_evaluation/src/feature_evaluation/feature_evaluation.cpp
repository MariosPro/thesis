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
#include "feature_evaluation/feature_evaluation.h"

namespace feature_evaluation
{
  /**
  @class MapEvaluator
  @brief Implements the OGM evaluations methods
  **/ 
  MapEvaluator::MapEvaluator(int argc, char** argv)
    {
      _mapEvaluationService =  _nh.advertiseService(
          "feature_evaluation/map_evaluation", &MapEvaluator::evaluationCallback, this);
    }

    /**
    @brief Service callback for performing map evaluation
    @param req [ogm_communications::ServerRequestEvaluation::Request&] The service request
    @param res [ogm_communications::ServerRequestEvaluation::Response&] The service response
    @return bool
    **/
    bool MapEvaluator::evaluationCallback(ogm_communications::ServerRequestEvaluation::Request& req,
      ogm_communications::ServerRequestEvaluation::Response& res)
    {
      _groundTruthMap = mapToMat(req.groundTruthMap);
      _slamMap = mapToMat(req.slamMap);
      _transform = req.transform;
    if (req.manualAlignment)
    {
      _alignment.alignMaps(_transform, _groundTruthMap, _slamMap);
    }

    bool s = req.manualAlignment;
    //ROS_INFO_STREAM("manual alignment=" << s);

    _mapUtils->findUsefulBoundaries(_groundTruthMap);
    _mapUtils->findUsefulBoundaries(_slamMap);

    if (req.binary)
    {
      for (int i = 0; i < _groundTruthMap.rows; i++)
        for (int j = 0; j < _groundTruthMap.cols; j++)
          if(_groundTruthMap.at<uchar>(i, j) == 127)
            _groundTruthMap.at<uchar>(i, j) = 255;

      for (int i = 0; i < _slamMap.rows; i++)
        for (int j = 0; j < _slamMap.cols; j++)
          if(_slamMap.at<uchar>(i, j) == 127)
            _slamMap.at<uchar>(i, j) = 255;

    }

    _params.detector = req.detector;
    _params.descriptor = req.descriptor;
    _params.matcher = req.matcher;
    _params.matchingMethod = req.matchingMethod;
    _params.closestPointMethod = req.closestPointMethod;
    _params.distNorm = req.distNorm;
    _params.matchingRatio = req.matchingRatio;
    _params.ransacReprjError = req.ransacReprjError;
    _params.scaleMapsBrushfire = req.scaleMapsBrushfire;
    _params.binary = req.binary;
    _params.manualAlignment = req.manualAlignment;
    _params.benchmarking = true;
    _params.morphologicalFiltering = req.morphologicalFiltering;
    _params.gaussianBlur1 = req.gaussianBlur1;
    _params.gaussianBlur2 = req.gaussianBlur2;
    _params.medianBlur1 = req.medianBlur1;
    _params.medianBlur2 = req.medianBlur2;
    _params.gaussianKernel1 = req.gaussianKernel1;
    _params.gaussianKernel2 = req.gaussianKernel2;
    _params.medianKernel1 = req.medianKernel1;
    _params.medianKernel2 = req.medianKernel2;
    _metric =  _metric_factory.createMetricInstance(req.method, _groundTruthMap, _slamMap);

    double totaltime = 0;
    for (int i = 0; i < 1; i++)
    {
      auto start = std::chrono::steady_clock::now();
      _metric->calculateMetric(_params);
      auto finish = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double> >(finish - start).count();
      totaltime += elapsed;
    }
    double meantime = totaltime /3;
    ROS_INFO_STREAM("EXECUTION TIME:" << meantime);
    res.result = _metric->getResult();
    //std::cout << "result" << res.result << std::endl;
    if(req.method == "FEATURES")
    {
      res.featuresImage = _metric->getFeaturesImage();
      res.initialMatchedImage = _metric->getInitialMatchedImage();
      res.finalMatchedImage = _metric->getFinalMatchedImage();
      res.mergedImage = _metric->getMergedImage();
      res.matches = _metric->getMatches();
      res.acceptance = _metric->getAcceptance();
      res.quality = _metric->getQuality();
      res.overlapArea = _metric->getOverlap();
      res.meanExTime = meantime;
    }

    return true;
  }

  /**
  @brief Method that converts nav_msgs/OccupancyGrid to cv::Mat
  @oaram map [const nav_msgs::OccupancyGrid&] the OccupancyGrid map
  @return void
  **/
  cv::Mat MapEvaluator::mapToMat(const nav_msgs::OccupancyGrid& map)
  {
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    
    if (map.info.height*map.info.width != map.data.size())
    {
        ROS_ERROR("Data size doesn't match width*height: width = %d, height = %d, data size = %zu", map.info.width, map.info.height, map.data.size());
    }

    for (size_t i = 0; i < map.info.height * map.info.width; i++)
    {
        if (map.data.at(i) == -1)
        {
            im.data[i] = 127;
        }
        else
        {
          im.data[i] = (100.0 - map.data.at(i)) / 100.0 * 255.0;
        }
    }
    cv::Mat flipped;
    cv::flip(im, flipped, 0);
    return flipped;
  }
}  // namespace feature_evaluation
