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
  @param std::string [detector] the featureDetector to be used
  @param std::string [descriptor] the DescriptorExtractor to be used
  @param std::string [matcher] the FeatureMatcher to be used
  @param std::string [distNorm] the distance norm to be used
  @return void
  **/

  FeatureMetrics::FeatureMetrics(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap,
                         std::string detector,
                         std::string descriptor,
                         std::string matcher,
                         std::string distNorm,
                         double matchingRatio,
                         double ransacReprjError)
                : Metric(groundTruthMap, slamMap)
  {
    _distNorm = distNorm;
    _detector = detector;
    _descriptor = descriptor;
    _matcherName = matcher;
    _matchingRatio = matchingRatio;
    _ransacReprjError = ransacReprjError;
    cv::initModule_nonfree();
    _featureDetector =  cv::FeatureDetector::create(_detector);
    if(_descriptor == "SIFT" || _descriptor == "SURF" || _descriptor == "BRIEF" ||
       _descriptor == "BRISK" || _descriptor == "FREAK" || _descriptor == "ORB")
    _descriptorExtractor = cv::DescriptorExtractor::create(_descriptor);
    
    if(_descriptor == "ALL CUSTOMS")
    {
      _customDescriptorExtractor.reserve(3);
      _customDescriptorExtractor.push_back(_descriptorFactory[0].create("RADIUS STATISTICS"));

      _customDescriptorExtractor.push_back(_descriptorFactory[1].create("CIRCLE INTERSECTIONS"));

      _customDescriptorExtractor.push_back(_descriptorFactory[2].create("MEAN RAYS"));
    }

    else
    {
     _customDescriptorExtractor.reserve(1);
    _customDescriptorExtractor.push_back(_descriptorFactory[0].create(_descriptor));
    }

    _matcher = cv::DescriptorMatcher::create(_matcherName);
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
    if(_descriptor == "SIFT" || _descriptor == "SURF" || _descriptor == "BRIEF" ||
       _descriptor == "BRISK" || _descriptor == "FREAK" || _descriptor == "ORB")
    {
      _descriptorExtractor->compute(_groundTruthMap, _groundTruthKeypoints, _groundTruthDescriptors);
      _descriptorExtractor->compute(_slamMap, _slamKeypoints, _slamDescriptors);
    }

    else
    {
      std::vector<cv::Mat> groundTruthDescriptors(_customDescriptorExtractor.size()),
                           slamDescriptors(_customDescriptorExtractor.size());

      for(int i = 0; i < _customDescriptorExtractor.size(); i++)
      {
        _customDescriptorExtractor[i]->compute(_slamMap, _slamKeypoints, &slamDescriptors[i]);
        _customDescriptorExtractor[i]->compute(_groundTruthMap, _groundTruthKeypoints, &groundTruthDescriptors[i]);
      ROS_INFO_STREAM("SLAM DESCRIPTORS[" <<i<<"]="<<slamDescriptors[i].size());
      }


      // Descriptors Concatenation
      cv::hconcat(slamDescriptors, _slamDescriptors);
      cv::hconcat(groundTruthDescriptors, _groundTruthDescriptors);
   }
 
    std::vector<std::vector<cv::DMatch> > matches12, matches21, matches;
    std::vector<cv::DMatch> crossCheckedMatches;
    std::vector< cv::DMatch > goodmatches;//, matches;
    //!< Matching descriptor vectors using a matcher
    //_matcher->match(_slamDescriptors, _groundTruthDescriptors, matches);

    ROS_INFO_STREAM("SLAM KEYPOINTS= " << _slamKeypoints.size());
    ROS_INFO_STREAM("SLAM DESCRIPTORS=" << _slamDescriptors.rows << " "  << _slamDescriptors.cols << " " << _slamDescriptors.type());
    ROS_INFO_STREAM("GROUND TRUTH KEYPOINTS= " << _groundTruthKeypoints.size());
    ROS_INFO_STREAM("GROUND TRUTH DESCRIPTORS=" << _groundTruthDescriptors.rows << " "  << _groundTruthDescriptors.cols << " " << _groundTruthDescriptors.type());
    ROS_INFO_STREAM("MATCHING RATIO=" << _matchingRatio);

    _matcher->knnMatch(_slamDescriptors, _groundTruthDescriptors, matches, 2);
 
    cv::KeyPoint a1, a2, b1, b2;
    goodmatches.clear();

    for (int i = 0; i < matches.size(); i++)
    {
      if (matches[i][0].distance < _matchingRatio * matches[i][1].distance)
      {
        goodmatches.push_back(matches[i][0]);
        a1 = _slamKeypoints[matches[i][0].queryIdx];
        b1 = _groundTruthKeypoints[matches[i][0].trainIdx];
        coord1.push_back(a1.pt);
        coord2.push_back(b1.pt);
        fil1.push_back(a1);
        fil2.push_back(b1);
      }
    }

/*    // find matching point pairs with same distance in both images*/
    //for (size_t i = 0; i < matches.size(); i++) 
    //{
      //a1 = _slamKeypoints[matches[i].queryIdx];
      //b1 = _groundTruthKeypoints[matches[i].trainIdx];

      ////if (matches[i].distance > 30)
        //[>continue;<]

      //for (size_t j = i + 1; j < matches.size(); j++) 
      //{
        //a2 = _slamKeypoints[matches[j].queryIdx];
        //b2 = _groundTruthKeypoints[matches[j].trainIdx];

//[>        if (matches[j].distance > 30)<]
          ////continue;

  //[>      if (fabs(cv::norm(a1.pt - a2.pt) - cv::norm(b1.pt - b2.pt)) > 5)// ||<]
            //////fabs(cv::norm(a1.pt - a2.pt) - cv::norm(b1.pt - b2.pt)) == 0)
          ////continue;

      //coord1.push_back(a1.pt);
      //coord2.push_back(b1.pt);
      //fil1.push_back(a1);
      //fil2.push_back(b1);
      //goodmatches.push_back(matches[j]);
    //}

    //coord1.push_back(a2.pt);
    //coord2.push_back(b2.pt);
    //fil1.push_back(a2);
    //fil2.push_back(b2);
  /*}*/
    ROS_INFO_STREAM("MATCHED KEYPOINTS=" << fil1.size() << " " << fil2.size() << " " << matches.size());
    ROS_INFO_STREAM("MAX RANSAC REPROJECTION ERROR=" << _ransacReprjError);

    std::vector <uchar> mask;
      //H = cv::estimateRigidTransform(coord2, coord1, true);
     H =  cv::findHomography(coord2, coord1, CV_RANSAC, _ransacReprjError, mask);
     if(H.empty())
      {
          ROS_WARN("H contain no data, cannot find valid transformation");
          return;
      }

    std::cout << "inliers = " << std::accumulate(mask.begin(), mask.end(), 0) << std::endl;
       /* for( size_t m = 0; m < matches12.size(); m++ )*/
    //{
        //bool findCrossCheck = false;
        //for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        //{
          //cv::DMatch forward = matches12[m][fk];

            //for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            //{
              //cv::DMatch backward = matches21[forward.trainIdx][bk];
                //if( backward.trainIdx == forward.queryIdx )
                //{
                    //crossCheckedMatches.push_back(forward);
                    //findCrossCheck = true;
                    //break;
                //}
            //}
            //if( findCrossCheck ) break;
        //}
    /*}*/
     //-- PS.- radiusMatch can also be used here.

    //goodmatches = crossCheckedMatches;
  /*  float ratio = 0.7;*/
  /*  for (int j = 0; j < matches.size(); j++)*/
    //{
      /*goodmatches.clear();*/
 /*     for(int i = 0; i < matches.size(); i++)*/
      //{
        //////if (matches[i][0].distance < ratio * matches[i][1].distance)
        ////{
          //goodmatches.push_back(matches[i][0]);
      ////-- Draw only "good" matches
    
        ////}
      /*}*/
      cv::Mat imgmatches;
      cv::drawMatches( _slamMap, _slamKeypoints, _groundTruthMap, _groundTruthKeypoints,
                   goodmatches, imgmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //[>  cv::drawMatches(_slamMap, fil1, _groundTruthMap, fil2, goodmatches, imgmatches, cv::Scalar::all(-1),cv::Scalar::all(-1));<]
      ////-- Show detected matches
      imshow( "Matches", imgmatches );
      cv::waitKey(1000);
    //}

 /*    double max_dist = 0; double min_dist = 100;*/
    ////-- Quick calculation of max and min distances between keypoints
    //for( int i = 0; i < _groundTruthDescriptors.rows; i++ )
    //{ double dist = matches12[i][0].distance;
      //if( dist < min_dist ) min_dist = dist;
      //if( dist > max_dist ) max_dist = dist;
    //}

    //ROS_INFO("-- Max dist : %f \n", max_dist );
    //ROS_INFO("-- Min dist : %f \n", min_dist );

   //[> -- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,<]
    ////-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //[>-- small)<]
    //for( int i = 0; i <_groundTruthDescriptors.rows; i++ )
    //{ if( matches12[i][0].distance <= cv::max(2*min_dist, 300.00) )
      //{ goodmatches.push_back( matches12[i][0]); }
    /*}*/
 
   cv::Mat image(_groundTruthMap.size(), _groundTruthMap.type());
   cv::Mat image1(_slamMap.size(), _slamMap.type(), 127);
   cv::warpPerspective(_groundTruthMap, image, H, image.size(), cv::INTER_NEAREST, IPL_BORDER_CONSTANT, cv::Scalar::all(127));
   //cv::warpAffine(_groundTruthMap, image, H, image.size());

   for (int i = 0; i < image1.rows; i++)
     for (int j = 0; j < image1.cols; j++)
      if(image.at<unsigned char>(i, j) != 127)
        image1.at<unsigned char>(i, j) = _slamMap.at<unsigned char>(i, j);

    cv::imshow("GroundTruthMap Transformed", image);
    cv::imshow ("SLamMap Cropped", image1);
    cv::waitKey(1000);
    _omseMetric =  new OmseMetric(image, image1, "Brushfire", _distNorm);
    _omseMetric->calculateMetric();
 
    // blend image1 onto the transformed image2
    addWeighted(image,.5, _slamMap, .5, 0.0, image);
    cv::imshow("MergedImage", image);

    for( int i = 0; i < (int)goodmatches.size(); i++ )
    {
     /* ROS_INFO( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  --Distance %f  \n", i, goodmatches[i].queryIdx, goodmatches[i].trainIdx, goodmatches[i].distance );*/
    }
    cv::Mat img_keypoints_1, img_keypoints_2;
    cv::drawKeypoints( _groundTruthMap, _groundTruthKeypoints, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::drawKeypoints( _slamMap, _slamKeypoints, img_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
   imshow("Keypoints 1", img_keypoints_1);
   imshow("Keypoints 2", img_keypoints_2);
    cv::waitKey(1000);
    _result = _omseMetric->getResult();
  }
}  // namespace ogm_evaluation

