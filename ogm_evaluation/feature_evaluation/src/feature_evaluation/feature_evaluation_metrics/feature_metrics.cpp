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
#include "feature_evaluation/feature_evaluation_metrics/feature_metrics.h"

namespace feature_evaluation
{
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

  FeatureMetrics::FeatureMetrics(const cv::Mat& groundTruthMap,
                         const cv::Mat& slamMap)
                  : Metric(groundTruthMap, slamMap)
  {
      ROS_INFO_STREAM("Created FeatureMetrics Instance");

  }

  /**
  @brief calculate the feature metric.
  @return void
  **/
  void FeatureMetrics::calculateMetric(Parameters params)
  {
     _result = 0;
     cv::initModule_nonfree();

     cv::Mat slamMap, groundTruthMap;

     _slamMap.copyTo(slamMap);
     _groundTruthMap.copyTo(groundTruthMap);

    if(params.gaussianBlur2)
      cv::GaussianBlur(slamMap, slamMap, cv::Size(params.gaussianKernel2, params.gaussianKernel2), 0, 0);
    if(params.medianBlur2)
      cv::medianBlur(slamMap, slamMap, params.medianBlur2);
    if(params.gaussianBlur1)
      cv::GaussianBlur(groundTruthMap, groundTruthMap, cv::Size(params.gaussianKernel1, params.gaussianKernel1), 0, 0 );
    if(params.medianBlur1)
      cv::medianBlur(groundTruthMap, groundTruthMap, params.medianBlur1);

     if(params.benchmarking)
     {
       _package_path = ros::package::getPath("ogm_server");
       _results_dir = _package_path + "/benchmarking_results/";
     }

     _featureDetector =  cv::FeatureDetector::create(params.detector);

    if(params.descriptor == "SIFT" || params.descriptor == "SURF" || params.descriptor == "BRIEF" ||
       params.descriptor == "BRISK" || params.descriptor == "FREAK" || params.descriptor == "ORB")
    _descriptorExtractor = cv::DescriptorExtractor::create(params.descriptor);

    if(params.descriptor == "ALL CUSTOMS")
    {
      _customDescriptorExtractor.reserve(3);
      _customDescriptorExtractor.push_back(_descriptorFactory[0].create("RADIUS STATISTICS"));

      _customDescriptorExtractor.push_back(_descriptorFactory[1].create("CIRCLE INTERSECTIONS"));

      _customDescriptorExtractor.push_back(_descriptorFactory[2].create("MEAN RAYS"));
    }

    else
    {
      _customDescriptorExtractor.reserve(1);
      _customDescriptorExtractor.push_back(_descriptorFactory[0].create(params.descriptor));
    }

    _matcher = cv::DescriptorMatcher::create(params.matcher);

    //!< scale the two Maps
    double slamMeanDist, groundTruthMeanDist;

    int** brushfire = new int*[groundTruthMap.rows];
      for(int i = 0; i < groundTruthMap.rows; i++)
        brushfire[i] = new int[groundTruthMap.cols];

    int** brushfire1 = new int*[slamMap.rows];
      for(int i = 0; i < slamMap.rows; i++)
        brushfire1[i] = new int[slamMap.cols];

    if(params.scaleMapsBrushfire)  
    {
      _mapUtils.brushfireSearch(_groundTruthMap, brushfire);
      groundTruthMeanDist = _mapUtils.meanBrushfireDistance(_groundTruthMap, brushfire);
      _mapUtils.brushfireSearch(_slamMap, brushfire1);
      slamMeanDist = _mapUtils.meanBrushfireDistance(_slamMap, brushfire1);

      double scalingFactor = groundTruthMeanDist / slamMeanDist;

      // resize slam produced map using meanBrushfireDistance
      cv::resize(_slamMap, _slamMap, cv::Size(), scalingFactor, scalingFactor, cv::INTER_NEAREST);
      cv::resize(slamMap, slamMap, cv::Size(), scalingFactor, scalingFactor, cv::INTER_NEAREST);

      ROS_INFO_STREAM("SCALING FACTOR=" << scalingFactor);
      std::cout << "SLAM AFTER RESIZE=" << slamMap.size() << std::endl;
    }

    //!< detect _slamKeypoints
    _featureDetector->detect(groundTruthMap, _groundTruthKeypoints);
    _featureDetector->detect(slamMap, _slamKeypoints);

    //!< extract Descriptors for each detected keypoint
    if(params.descriptor == "SIFT" || params.descriptor == "SURF" || params.descriptor == "BRIEF" ||
       params.descriptor == "BRISK" || params.descriptor == "FREAK" || params.descriptor == "ORB")
    {
      _descriptorExtractor->compute(groundTruthMap, _groundTruthKeypoints, _groundTruthDescriptors);
      _descriptorExtractor->compute(slamMap, _slamKeypoints, _slamDescriptors);
    }

    else
    {
      std::vector<cv::Mat> groundTruthDescriptors(_customDescriptorExtractor.size()),
                           slamDescriptors(_customDescriptorExtractor.size());

      for(int i = 0; i < _customDescriptorExtractor.size(); i++)
      {
        _customDescriptorExtractor[i]->compute(slamMap, _slamKeypoints, &slamDescriptors[i]);
        _customDescriptorExtractor[i]->compute(groundTruthMap, _groundTruthKeypoints, &groundTruthDescriptors[i]);
        ROS_INFO_STREAM("SLAM DESCRIPTORS[" <<i<<"]="<<slamDescriptors[i].size());
      }

      // Descriptors Concatenation
      cv::hconcat(slamDescriptors, _slamDescriptors);
      cv::hconcat(groundTruthDescriptors, _groundTruthDescriptors);
   }

    std::vector<std::vector<cv::DMatch> > matches12, matches21, matches;
    std::vector<cv::DMatch> crossCheckedMatches;
    std::vector< cv::DMatch > filteredMatches;//, matches;

    ROS_INFO_STREAM("SLAM KEYPOINTS= " << _slamKeypoints.size());
    ROS_INFO_STREAM("SLAM DESCRIPTORS=" << _slamDescriptors.rows << " "  << _slamDescriptors.cols << " " << _slamDescriptors.type());
    ROS_INFO_STREAM("GROUND TRUTH KEYPOINTS= " << _groundTruthKeypoints.size());
    ROS_INFO_STREAM("GROUND TRUTH DESCRIPTORS=" << _groundTruthDescriptors.rows << " "  << _groundTruthDescriptors.cols << " " << _groundTruthDescriptors.type());
    ROS_INFO_STREAM("MATCHING RATIO=" << _matchingRatio);

   //!< draw Keypoints
   cv::Mat img_keypoints_1, img_keypoints_2;
   cv::drawKeypoints( groundTruthMap, _groundTruthKeypoints, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
   cv::drawKeypoints( slamMap, _slamKeypoints, img_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
   imshow("Keypoints 1", img_keypoints_1);
   imshow("Keypoints 2", img_keypoints_2);
   cv::waitKey(1000);

    std::vector<cv::KeyPoint> slamMatchedKeyPoints, groundTruthMatchedKeyPoints;
    std::vector<cv::Point2f>  slamMatchedCoords, groundTruthMatchedCoords;
    filteredMatches.clear();
    slamMatchedKeyPoints.clear();
    groundTruthMatchedKeyPoints.clear();
    slamMatchedCoords.clear();
    groundTruthMatchedCoords.clear();

    //!< Matching descriptor vectors using one of the following matching methods

    if (params.matchingMethod == "SIMPLE") 
      simpleMatching(_slamDescriptors, _groundTruthDescriptors, filteredMatches);
    else if(params.matchingMethod ==  "RATIO")
      ratioTest(_slamDescriptors, _groundTruthDescriptors, filteredMatches);
    else if(params.matchingMethod == "CROSSCHECK")
      crossCheckMatching(_slamDescriptors, _groundTruthDescriptors, filteredMatches, 1);
    else if(params.matchingMethod == "K-NN")   
      knnMatching(_slamDescriptors, _groundTruthDescriptors, filteredMatches, 5);// _groundTruthDescriptors.rows);
    else
      ROS_ERROR("No such matching method");
    if(filteredMatches.size() == 0)
    {
      ROS_WARN("No matches found using this method");
      return;
    }
    //!< draw matches
    cv::Mat imgmatches, imgmatches1, imgmatches2;
    cv::drawMatches( slamMap, _slamKeypoints, groundTruthMap, _groundTruthKeypoints,
                   filteredMatches, imgmatches1, cv::Scalar::all(-1), cv::Scalar::all(-1));

    //!< Show detected matches
/*    imshow("Initial Matches", imgmatches1);*/
    /*cv::waitKey(1000);*/

    std::vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
    for( size_t i = 0; i < filteredMatches.size(); i++ )
    {
        queryIdxs[i] = filteredMatches[i].queryIdx;
        trainIdxs[i] = filteredMatches[i].trainIdx;
    }

    cv::KeyPoint::convert(_slamKeypoints, slamMatchedCoords, queryIdxs);
    cv::KeyPoint::convert(_groundTruthKeypoints, groundTruthMatchedCoords, trainIdxs);

    //!< evaluate matches through custom descriptors

   /* cv::Mat slamEvalDescriptors, groundTruthEvalDescriptors;*/
    //cv::Ptr<feature_evaluation::DescriptorExtractor> customDescriptorEvaluator;

    //customDescriptorEvaluator = _descriptorFactory[1].create("CIRCLE INTERSECTIONS");
    //customDescriptorEvaluator->compute(groundTruthMap, groundTruthMatchedKeyPoints, &groundTruthEvalDescriptors);
    //customDescriptorEvaluator->compute(slamMap, slamMatchedKeyPoints, &slamEvalDescriptors);

    //std::vector<cv::Point2f>  evalcoord1, evalcoord2;
    //std::vector<cv::KeyPoint> evalfil1, evalfil2;
    //std::vector<cv::DMatch> evalmatches;
    //int counter;
    /*evalmatches.clear();*/
    
/*    for (int i = 0; i < slamEvalDescriptors.rows; i++)*/
    //{
      //counter = 0;

      //for (int j = 0; j < slamEvalDescriptors.cols; j++)
      //{
        //if (abs(slamEvalDescriptors.at<float>(i, j) - groundTruthEvalDescriptors.at<float>(i, j)) <= 1)
          //counter++;
      //}

      ////if (counter >= 4)
      //{
        //evalcoord1.push_back(slamMatchedKeyPoints[i].pt);
        //evalcoord2.push_back(groundTruthMatchedKeyPoints[i].pt);
        //evalfil1.push_back(slamMatchedKeyPoints[i]);
        //evalfil2.push_back(groundTruthMatchedKeyPoints[i]);
        //evalmatches.push_back(filteredMatches[i]);
      //}
    /*}*/


    ROS_INFO_STREAM("MATCHED KEYPOINTS=" << slamMatchedKeyPoints.size() << " " << groundTruthMatchedKeyPoints.size() << " " << filteredMatches.size());
    ROS_INFO_STREAM("MATCHED COORDS=" << slamMatchedCoords.size() << " " << groundTruthMatchedCoords.size());
       //ROS_INFO_STREAM("MATCHED EVALUATED KEYPOINTS=" << evalfil1.size() << " " << evalfil2.size() << " " << evalmatches.size());

    ROS_INFO_STREAM("MAX RANSAC REPROJECTION ERROR=" << params.ransacReprjError);
/*for (int i = 0; i < slamMatchedCoords.size(); i++){*/
      //std::cout << "coords " << slamMatchedCoords[i] << " " << groundTruthMatchedCoords[i] << " ";
      //std::cout << "keypoints " << slamMatchedKeyPoints[i].pt << " " << groundTruthMatchedKeyPoints[i].pt << " ";}
        /*std::cout << std::endl;*/

    std::vector<uchar> mask;
    if(slamMatchedCoords.size() < 3)
    {
      ROS_WARN("affine transform needs at least 3 points to be computed");
      return;
    }
    double best_error;
    cv::Mat T;

/*    estimateTransform(groundTruthMatchedCoords, slamMatchedCoords,*/
                       //1000, _ransacReprjError, 3, mask, T, best_error);

  /*  int valid = estimateAffine2D(groundTruthMatchedCoords, slamMatchedCoords, T, mask, params.ransacReprjError);*/
    int valid = estimateAffinePartial2D(groundTruthMatchedCoords, slamMatchedCoords, T, mask, params.ransacReprjError);

    std::cout << "ransac result=" << valid << std::endl; 

    std::cout << "H = "<< std::endl << " "  << T << std::endl << std::endl;

   if(T.empty() || std::count( mask.begin(), mask.end(), 1) < 3)
   {
     ROS_WARN("H contain no data, cannot find valid transformation");
     return;
   }


   std::cout << " RANSAC inliers = " << std::accumulate(mask.begin(), mask.end(), 0) << std::endl;
 /*  std::vector<cv::Point2f> inliersCoords1, inliersCoords2;*/
   //for (int i = 0; i < mask.size(); i++)
   //{
     //if((int)mask[i] == 1)
     //{
       //inliersCoords1.push_back(slamMatchedCoords[i]);
       //inliersCoords2.push_back(groundTruthMatchedCoords[i]);
     //}
   /*}*/
  
   //!< draw matches
   cv::drawMatches( slamMap, _slamKeypoints, groundTruthMap, _groundTruthKeypoints,
                   filteredMatches, imgmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   reinterpret_cast<const std::vector<char>&> (mask), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
   
   _matchedImage = imgmatches.clone();
    ////-- Show detected matches
    if(!params.benchmarking)
    {
      imshow("RANSAC Matches", imgmatches);
      cv::waitKey(1000);
    }

   /*H = cv::estimateRigidTransform(inliersCoords2, inliersCoords1, false);*/

   //if(H.empty())
   //{
     //ROS_WARN("H contain no data, cannot find valid transformation");
     //return;
   /*}*/
 
  
    //}
    
   cv::Mat image(_slamMap.size(), _slamMap.type());
   cv::Mat image1(_slamMap.size(), _slamMap.type(), 127);
   
   std::cout << "H.type=" << T.type() << std::endl;

 /*  double a =T.at<double>(0, 0);*/
   //double b = T.at<double>(0, 1);
   //double c = T.at<double>(0, 2);
   //double d = T.at<double>(1, 0);
   //cv::Mat_< double > M(2, 3);
   //M << a, -b, c,
        //b,  a, d;
   //std::cout << "M.type=" << M.type() << std::endl;
   /*std::cout << "M = "<< std::endl << " "  << M << std::endl << std::endl;*/

   cv::warpAffine(_groundTruthMap, image, T, image.size(), cv::INTER_NEAREST, IPL_BORDER_CONSTANT, cv::Scalar::all(127));
  
   for (int i = 0; i < image1.rows; i++)
     for (int j = 0; j < image1.cols; j++)
      if(image.at<unsigned char>(i, j) != 127)
        image1.at<unsigned char>(i, j) = _slamMap.at<unsigned char>(i, j);
  if(!params.benchmarking)
  {
    cv::imshow("GroundTruthMap Transformed", image);
    cv::imshow ("SLamMap Cropped", image1);
    cv::waitKey(1000);
  }
    _omseMetric =  new OmseMetric(image, image1);
    params.closestPointMethod = "Brushfire";
    _omseMetric->calculateMetric(params);
 
    std::cout << "image1 type " << image1.type() << " " << image1.channels() << std::endl;
    std::cout << "image type " << image.type() << " " << image.channels() << std::endl;
    
    //!< blend slamMap onto the transformed groundTruthMap
    addWeighted(image, .5, slamMap, .5, 0.0, image);
     std::cout << "image type " << image.type() << " " << image.channels() << std::endl;

    _mergedImage = image.clone();
    if(!params.benchmarking)
    {
      cv::imshow("MergedImage", image);
      cv::waitKey(1000);
    }

    std::cout << mask.size() << std::endl;
    int counter = 0;
    for( int i = 0; i < filteredMatches.size(); i++ )
    {
      //std::cout << (int)mask[i]<<std::endl;
      if( int(mask[i]) == 1)
      {
      /*ROS_INFO( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  --Distance %f  \n", i, filteredMatches[i].queryIdx, filteredMatches[i].trainIdx, filteredMatches[i].distance );*/
      /*counter++;*/
      }
    }

    //std::cout << counter << std::endl;
    _result = _omseMetric->getResult();
    //_result = 0;
}

void FeatureMetrics::crossCheckMatching(const cv::Mat& descriptors1,
                                        const cv::Mat& descriptors2,
                                        std::vector<cv::DMatch>& filteredMatches12,
                                        int knn)
{
    filteredMatches12.clear();
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    _matcher->knnMatch( descriptors1, descriptors2, matches12, knn);
    _matcher->knnMatch( descriptors2, descriptors1, matches21, knn);
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
          cv::DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
              cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}

void FeatureMetrics::simpleMatching(const cv::Mat& descriptors1,
                                    const cv::Mat& descriptors2,
                                    std::vector<cv::DMatch>& matches12)
{
    _matcher->match(descriptors1, descriptors2, matches12);
  
}
 void  FeatureMetrics::ratioTest(const cv::Mat& descriptors1, 
                                 const cv::Mat& descriptors2,
                                 std::vector<cv::DMatch>& filteredMatches)
{
     std::vector<std::vector<cv::DMatch> > matches;

     _matcher->knnMatch(descriptors1, descriptors2, matches, 2);

    for (size_t i = 0; i < matches.size(); i++)
    {
      if (matches[i][0].distance < _matchingRatio * matches[i][1].distance)
      {
        filteredMatches.push_back(matches[i][0]);
      /*  ROS_INFO_STREAM("FILTERED MATCHES idx " << matches[i][0].queryIdx << " " << matches[i][0].trainIdx);*/
        /*ROS_INFO_STREAM("s " << _slamKeypoints[matches[i][0].queryIdx].pt << " " << _groundTruthKeypoints[matches[i][0].trainIdx].pt);*/
      }
    }
}

void FeatureMetrics::knnMatching(const cv::Mat& descriptors1,
                                    const cv::Mat& descriptors2,
                                    std::vector<cv::DMatch>& filteredMatches,
                                    int knn)
{
    std::vector<std::vector<cv::DMatch> > matches12;
    _matcher->knnMatch( descriptors1, descriptors2, matches12, knn);
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
          cv::DMatch match = matches12[m][fk];
          if(match.distance < 250)
          filteredMatches.push_back(match);
        }
    }
}


void FeatureMetrics::estimateTransform(const std::vector<cv::Point2f>& coords1, const std::vector<cv::Point2f>& coords2,
                       int nIters, double thresh, int minNpoints,
                       std::vector<uchar>& inliers,
                       cv::Mat& best_model, double& best_error)
{
  /*  The input to the algorithm is:*/
  //n - the number of random points to pick every iteration in order to create the transform. I chose n = 3 in my implementation.
  //k - the number of iterations to run
  //t - the threshold for the square distance for a point to be considered as a match
  //d - the number of points that need to be matched for the transform to be valid
  /*image1_points and image2_points - two arrays of the same size with points. Assumes that image1_points[x] is best mapped   to image2_points[x] accodring to the computed features.*/

  std::vector<unsigned int> indices(coords1.size());
  std::vector<cv::Point2f> points1(3), points2(3), coords1t;
  std::vector<uchar> mask(coords1.size());
  std::cout <<"mask size=" << mask.size() << std::endl;

  //inliers.reserve(coords1.size());
  cv::Mat model;
  int consensus_set;
  double error, total_error;
  best_error = std::numeric_limits<double>::infinity();

  for (int i = 0; i < coords1.size(); i++)
    indices[i] = i;

  int counter = 0;
  while(counter < nIters)
  {
    //ROS_INFO_STREAM("ITER=" << i);
    // pick n=3 random points
    std::random_shuffle(indices.begin(), indices.end());
    for (int k = 0; k < 3; k++)
    {
      points1[k] = coords1[indices[k]];
      points2[k] = coords2[indices[k]];
      //std::cout << "picked " << indices[k] << std::endl;
    }
    //find transform
    model = cv::getAffineTransform(points1, points2);
    consensus_set = 0;
    total_error = 0;
    cv::transform(coords1, coords1t, model);

    // find inliers (where err < ransacReprjError)
    for (int j = 0; j < coords1.size(); j++)
    {
      error = cv::pow(cv::norm(coords2[j] - coords1t[j]), 2.0);
      int f = error <= (thresh * thresh);
      mask[j] = f;
      if(f)
      {

  /*        std::cout << "mask" << (int)mask[j] << std::endl;*/
        /*ROS_INFO_STREAM("ERROR=" << error );*/
        consensus_set++;
        total_error += error;
      }

    }

  /*     std::cout <<"mask size=" << mask.size() << std::endl;*/
    //std::cout << " RANSAC inliers = " << std::accumulate(mask.begin(), mask.end(), 0) << std::endl;
    /*ROS_INFO_STREAM("CONSENSUS_SET=" << consensus_set);*/
    
    if(consensus_set > std::max(minNpoints,std::accumulate(inliers.begin(), inliers.end(), 0)))//   && total_error < best_error)
    {
      //best_model = model;
      best_error = total_error;
      inliers.clear();
      inliers.insert(inliers.end(), mask.begin(), mask.end());
      std::vector<cv::Point2f> inliersCoords1, inliersCoords2;
      for (int l = 0; l < mask.size(); l++)
       {
         if((int)mask[l] == 1)
         {
           inliersCoords1.push_back(coords2[l]);
           inliersCoords2.push_back(coords1[l]);
         }
       }

      H = cv::estimateRigidTransform(inliersCoords2, inliersCoords1, false);
      
      if(!H.empty())
      {
        best_model = H;
      }
      std::cout << "best_model = "<< std::endl << " "  << best_model << std::endl << std::endl;
    }
/*    if (best_model.empty())*/
    //{
      //nIters++;
    /*}*/
    counter++;
  }

}

  sensor_msgs::Image FeatureMetrics::getMatchedImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _matchedImage;
    std::cout << _matchedImage.type() << " " << _matchedImage.channels() << std::endl;
    out_msg.toImageMsg(temp);
    return temp;
  }

  sensor_msgs::Image FeatureMetrics::getMergedImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _mergedImage;
    std::cout << _mergedImage.type() << " " << _mergedImage.channels() << std::endl;
    out_msg.toImageMsg(temp);
    return temp;
  }
}
