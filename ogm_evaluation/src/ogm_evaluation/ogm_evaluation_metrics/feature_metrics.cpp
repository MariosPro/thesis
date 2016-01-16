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
#include "ogm_evaluation/ogm_evaluation_metrics/feature_metrics.h"

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

  FeatureMetrics::FeatureMetrics(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap,
                         int method,
                         int distNorm)
                : Metric(groundTruthMap, slamMap)
  {
    _omse_method = method;
    _distNorm = distNorm;
    cv::initModule_nonfree();
    _featureDetector =  cv::FeatureDetector::create("SIFT");
    _descriptorExtractor = cv::DescriptorExtractor::create("SIFT");
    _matcher = cv::DescriptorMatcher::create("BruteForce");
    ROS_INFO_STREAM("Created FeatureMetrics Instance");

  }

  /**
  @brief calculate the feature metric.
  @return void
  **/
  void FeatureMetrics::calculateMetric()
  {
    //!< detect _slamKeypoints
    _featureDetector->detect(_groundTruthMap, _groundTruthKeypoints);
    _featureDetector->detect(_slamMap, _slamKeypoints);

    //!< extract Descriptors for each detected keypoint
    _descriptorExtractor->compute(_groundTruthMap, _groundTruthKeypoints, _groundTruthDescriptors);
    _descriptorExtractor->compute(_slamMap, _slamKeypoints, _slamDescriptors);
    
    std::vector<std::vector<cv::DMatch> > matches;
    //!< Matching descriptor vectors using a matcher
    _matcher->knnMatch(_groundTruthDescriptors, _slamDescriptors, matches, 2);

   /* double max_dist = 0; double min_dist = 100;*/
    ////-- Quick calculation of max and min distances between keypoints
    //for( int i = 0; i < _groundTruthDescriptors.rows; i++ )
    //{ double dist = matches[i].distance;
      //if( dist < min_dist ) min_dist = dist;
      //if( dist > max_dist ) max_dist = dist;
    //}

    //ROS_INFO("-- Max dist : %f \n", max_dist );
    //ROS_INFO("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< cv::DMatch > goodmatches;

    float ratio = 0.6;

    for (int i = 0; i <  matches.size(); i++)
    {
      if (matches[i][0].distance < ratio * matches[i][1].distance)
      {
        goodmatches.push_back(matches[i][0]);
      }
    }

   /* for( int i = 0; i <_groundTruthDescriptors .rows; i++ )*/
    //{ if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
      //{ goodmatches.push_back( _matches[i]); }
    /*}*/
 
    //-- Draw only "good" matches
    cv::Mat imgmatches;
    cv::drawMatches( _groundTruthMap, _groundTruthKeypoints, _slamMap, _slamKeypoints,
                 goodmatches, imgmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Matches", imgmatches );

    for( int i = 0; i < (int)goodmatches.size(); i++ )
    {
      ROS_INFO( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  --Distance %f  \n", i, goodmatches[i].queryIdx, goodmatches[i].trainIdx, goodmatches[i].distance );
    }
    cv::waitKey(30);
    _result = 0;
  }
}  // namespace ogm_evaluation

