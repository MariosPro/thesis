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

#include <string>
#include "feature_evaluation/feature_metric_factory.h"
#include "feature_evaluation/feature_evaluation_metrics/omse.h"
#include "feature_evaluation/feature_evaluation_metrics/feature_metrics.h"

namespace feature_evaluation
{
   /**
   * @brief The main factory method that creates the different metrics
   * @param name [std::string] The name of the metric
   * @param groundTruthMap [const cv::Mat&] the ground truth map
   * @oaram slamMap [const cv::Mat&] the slam map
   * that will be used.
   */
  boost::shared_ptr<Metric> MetricFactory::createMetricInstance(std::string name, 
                                              const cv::Mat& groundTruthMap,
                                              const cv::Mat& slamMap)
  {
    if (name == "OMSE") return boost::shared_ptr<Metric>(new OmseMetric(groundTruthMap, slamMap));
    else if (name == "FEATURES") return boost::shared_ptr<Metric>(new FeatureMetrics(groundTruthMap, slamMap));

    else
      {
        ROS_FATAL_STREAM("[feature_evaluation]: Invalid metric method!"
            << " Detection cannot continue!");
        return boost::shared_ptr<Metric>();
      }
      return boost::shared_ptr<Metric>();
  }
}  // namespace feature_evaluation
