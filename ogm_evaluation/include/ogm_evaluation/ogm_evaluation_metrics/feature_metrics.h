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

#ifndef FEATURE_METRICS_H
#define FEATURE_METRICS_H

#include <numeric>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <ros/package.h>
#include "ogm_communications/MapPose.h"
#include "ogm_evaluation/ogm_evaluation_metrics/metric_base.h"
#include "ogm_evaluation/ogm_evaluation_descriptors/descriptor_extractor.h"
#include "ogm_evaluation/ogm_evaluation_descriptors/descriptor_factory.h"
#include "ogm_evaluation/ogm_evaluation_metrics/omse.h"
#include "ogm_evaluation/ogm_evaluation_utils/affine2DEstimator.h"
#include "ogm_evaluation/ogm_evaluation_utils/maps_alignment.h"
#include "ogm_evaluation/ogm_evaluation_utils/map_utils.h"

/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  /**
  @class FeatureMetrics
  @brief A class that provides Metrics Implementation for various feature Descriptors.
  Inherits publicly Metric
  **/ 
  class FeatureMetrics : public Metric
  {
    public:

      /**
      @brief Default Constructor
      @param groundTruthMap [const cv::Mat& ] the ground truth map
      @param slamMap[const cv::Mat&] the slam produced Map
      @param std::string [detector] the featureDetector to be used
      @param std::string [descriptor] the DescriptorExtractor to be used
      @param std::string [matcher] the FeatureMatcher to be used
      @param std::string [distNorm] the distance norm to be used
      @param std::string [matchingMethod] the matching method to be used
      @param double [matchingRatio] the matching ratio if ratioTest is gonna be used
      @param double [ransacReprjError] the maximum allowed ransac reprojection error
      @return void
      **/
      FeatureMetrics(const cv::Mat& groundTruthMap,
                 const cv::Mat& slamMap);

      /**
      @brief Default destructor
      @return void
      **/
      virtual ~FeatureMetrics(void) 
      {
      }

      /**
      @brief calculate the obstacle metric.
      @return void
      **/
      virtual void calculateMetric(Parameters params);

      void crossCheckMatching(const cv::Mat& descriptors1,
                              const cv::Mat& descriptors2,
                              std::vector<cv::DMatch>& filteredMatches12,
                              int knn);


      void simpleMatching(const cv::Mat& descriptors1,
                    const cv::Mat& descriptors2,
                    std::vector<cv::DMatch>& matches12);
                  
      void ratioTest(const cv::Mat& descriptors1,
                     const cv::Mat& descriptors2, 
                     std::vector<cv::DMatch>& filteredMatches);
      
      void knnMatching(const cv::Mat& descriptors1,
                       const cv::Mat& descriptors2,
                       std::vector<cv::DMatch>& filteredMatches,
                       int knn);

      void estimateTransform(const std::vector<cv::Point2f>& coords1,
                            const std::vector<cv::Point2f>& coords2,
                            int nIters, double thresh, int minNpoints,
                            std::vector<uchar>& inliers,
                            cv::Mat& best_model, double& best_error);

      sensor_msgs::Image getMatchedImage();

      sensor_msgs::Image getMergedImage();

    private:

      //!< the Detector name to be created
      std::string _detector;

      //!< the Descriptor name to be created
      std::string _descriptor;

      //!< the matcher name to be created
      std::string _matcherName;

      //!< the distance norm to be used
      std::string _distNorm;
 
      //!< the matching method be used
      std::string _matchingMethod;

      //!< the matchingRatio to be used for discarding matches
      double _matchingRatio;

      //!< the max ransac Reprojection error for validating inliers
      double _ransacReprjError;

      //!< flag indicating if scaling to be performed (using
      //meanBrushfireDistance)
      bool _scaleMapsBrushfire;

      //!< the instance of opencv's FeatureDetector class to be used
      cv::Ptr<cv::FeatureDetector> _featureDetector;

      //!< the instance of opencv's DescriptorExtractor class to be used
      cv::Ptr<cv::DescriptorExtractor> _descriptorExtractor;
      
      //!< the instance of custom's DescriptorExtractor class to be used
      std::vector<cv::Ptr<ogm_evaluation::DescriptorExtractor> > _customDescriptorExtractor;

      //!< the Descriptors Factory
      std::vector<DescriptorFactory> _descriptorFactory;

      //!< the instance of opencv's DescriptorMatcher class to be used
      cv::Ptr<cv::DescriptorMatcher> _matcher;

      //!< the groundTruthMap feature keypoints
      std::vector<cv::KeyPoint> _groundTruthKeypoints;
      
      //!< the slamMap feature keypoints
      std::vector<cv::KeyPoint> _slamKeypoints;

      //!< the groundTruthMap feature descriptors
      cv::Mat _groundTruthDescriptors;

      //!< the slamMap feature descriptors
      cv::Mat _slamDescriptors;
     
     //!< the Matching descriptor vectors
     std::vector< cv::DMatch > _matches;
    
     std::vector<cv::KeyPoint> fil1,fil2;

     std::vector<cv::Point2f>  coord1,coord2;

     Metric* _omseMetric;

     Alignment _alignment;

     MapUtils _mapUtils;

     ogm_communications::MapPose _transform;

     cv::Mat H;

     std::string _package_path;

     std::string _results_dir;

     cv::Mat _matchedImage;
     
     cv::Mat _mergedImage;
  };
}
#endif
