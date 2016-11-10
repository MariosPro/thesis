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
#ifndef OGM_METRIC_FACTORY_H
#define OGM_METRIC_FACTORY_H

#include <string>
#include "feature_evaluation/feature_evaluation_metrics/metric_base.h"
#include "ogm_communications/MapPose.h"
/**
@namespace feature_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace feature_evaluation
{
   /**
   * @class MetricFactory
   * @brief The class that is used to produce the metrics.
   */
   class MetricFactory
   {
       public:
       /**
       * @brief Default Constructor
       */
       MetricFactory()
       {
       };
      
      virtual ~MetricFactory()
      {
        std::cout << "destroying Metric Factory instance" << std::endl;
      };
      boost::shared_ptr<Metric> createMetricInstance(std::string name,
                                    const cv::Mat& groundTruthMap,
                                    const cv::Mat& slamMap);


   };
}
#endif
