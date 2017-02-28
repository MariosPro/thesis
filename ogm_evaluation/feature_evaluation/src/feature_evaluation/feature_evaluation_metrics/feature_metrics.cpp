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
     _result = std::numeric_limits<double>::max();
     _inliers = 0;
     _acceptance = 0;
     _quality = 0;
     _overlap = 0;
     cv::initModule_nonfree();

     cv::Mat slamMap, groundTruthMap;

     _slamMap.copyTo(slamMap);
     _groundTruthMap.copyTo(groundTruthMap);

    if(params.gaussianBlur2)
      cv::GaussianBlur(slamMap, slamMap, cv::Size(params.gaussianKernel2, params.gaussianKernel2), 0, 0);
    if(params.medianBlur2)
      cv::medianBlur(slamMap, slamMap, params.medianKernel2);
    if(params.gaussianBlur1)
      cv::GaussianBlur(groundTruthMap, groundTruthMap, cv::Size(params.gaussianKernel1, params.gaussianKernel1), 0, 0 );
    if(params.medianBlur1)
      cv::medianBlur(groundTruthMap, groundTruthMap, params.medianKernel1);
    if(params.benchmarking)
     {
       _package_path = ros::package::getPath("ogm_server");
       _results_dir = _package_path + "/benchmarking_results/";
     }

     if(params.morphologicalFiltering)
    {
       _mapUtils->morphologicalFiltering(_groundTruthMap);
       _mapUtils->morphologicalFiltering(_slamMap);
    }

     _featureDetector =  cv::FeatureDetector::create(params.detector);

    if(params.descriptor == "SIFT" || params.descriptor == "SURF" || params.descriptor == "BRIEF" ||
       params.descriptor == "BRISK" || params.descriptor == "FREAK" || params.descriptor == "ORB")
      _descriptorExtractor = cv::DescriptorExtractor::create(params.descriptor);

    if(params.descriptor == "ALL CUSTOMS")
    {
      _customDescriptorExtractor.reserve(3);
      _customDescriptorExtractor.push_back(_descriptorFactory[0].create("ANNULAR STATISTICS"));
      _customDescriptorExtractor.push_back(_descriptorFactory[1].create("CIRCLE INTERSECTIONS"));
      _customDescriptorExtractor.push_back(_descriptorFactory[2].create("MEAN RAYS"));
    }

    else if(params.descriptor == "ANNULAR+RAYS")
    {
      _customDescriptorExtractor.reserve(2);
      _customDescriptorExtractor.push_back(_descriptorFactory[0].create("ANNULAR STATISTICS"));
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
    if(params.scaleMapsBrushfire)  
    {
      int** brushfire = new int*[groundTruthMap.rows];
        for(int i = 0; i < groundTruthMap.rows; i++)
          brushfire[i] = new int[groundTruthMap.cols];

      int** brushfire1 = new int*[slamMap.rows];
        for(int i = 0; i < slamMap.rows; i++)
          brushfire1[i] = new int[slamMap.cols];

      _mapUtils->brushfireSearch(_groundTruthMap, brushfire);
      groundTruthMeanDist = _mapUtils->meanBrushfireDistance(_groundTruthMap, brushfire);
      _mapUtils->brushfireSearch(_slamMap, brushfire1);
      slamMeanDist = _mapUtils->meanBrushfireDistance(_slamMap, brushfire1);

      double scalingFactor = groundTruthMeanDist / slamMeanDist;

      for (int i = 0; i <groundTruthMap.rows; i++)
      delete[] brushfire[i];
      delete[] brushfire;

      for (int i = 0; i <slamMap.rows; i++)
      delete[] brushfire1[i];
      delete[] brushfire1;

      // resize slam produced map using meanBrushfireDistance
      cv::resize(_slamMap, _slamMap, cv::Size(), scalingFactor, scalingFactor, cv::INTER_NEAREST);
      cv::resize(slamMap, slamMap, cv::Size(), scalingFactor, scalingFactor, cv::INTER_NEAREST);

      ROS_INFO_STREAM("SCALING FACTOR=" << scalingFactor);
      std::cout << "SLAM AFTER RESIZE=" << slamMap.size() << std::endl;
    }

    //!< detect Keypoints
    _featureDetector->detect(groundTruthMap, _groundTruthKeypoints);
    _featureDetector->detect(slamMap, _slamKeypoints);
    if(_slamKeypoints.size() == 0  || _groundTruthKeypoints.size() == 0)
    {
      ROS_WARN("No keypoints extracted in one or neither of maps");
      return;
    }

  /*    float repeatability;*/
    //int correspCount;
    //cv::Mat H1to2;
    //std::vector<cv::Point2f> srcPoints,refPoints;
    //cv::KeyPoint::convert(_groundTruthKeypoints, srcPoints);
    //cv::KeyPoint::convert(_slamKeypoints, refPoints);
    //H1to2 = cv::findHomography( srcPoints, refPoints, CV_RANSAC, 1 );
    //cv::evaluateFeatureDetector(groundTruthMap, slamMap, H1to2, &_groundTruthKeypoints, &_slamKeypoints, repeatability, correspCount, _featureDetector);
    //std::cout<<"repeatability="<<repeatability<<" correspCount="<<correspCount<<" Keypoint 1st="
             /*<<_groundTruthKeypoints.size()<<" Keypoint 2st=" <<_slamKeypoints.size() << std::endl;*/

    //!< extract Descriptors for each detected keypoint
    if(params.descriptor == "SIFT" || params.descriptor == "SURF" || params.descriptor == "BRIEF" ||
       params.descriptor == "BRISK" || params.descriptor == "FREAK" || params.descriptor == "ORB")
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
        //ROS_INFO_STREAM("SLAM DESCRIPTORS[" <<i<<"]="<<slamDescriptors[i].size());
      }

      // Descriptors Concatenation
      cv::hconcat(slamDescriptors, _slamDescriptors);
      cv::hconcat(groundTruthDescriptors, _groundTruthDescriptors);
      cv::normalize(_slamDescriptors, _slamDescriptors, 0, 1, cv::NORM_MINMAX, CV_32F);
      cv::normalize(_groundTruthDescriptors, _groundTruthDescriptors, 0, 1, cv::NORM_MINMAX, CV_32F);

   }
    std::vector<std::vector<cv::DMatch> > matches12, matches21, matches;
    std::vector<cv::DMatch> crossCheckedMatches;
    std::vector< cv::DMatch > filteredMatches;//, matches;

    ROS_INFO_STREAM("SLAM KEYPOINTS= " << _slamKeypoints.size());
    ROS_INFO_STREAM("SLAM DESCRIPTORS=" << _slamDescriptors.rows << " "  << _slamDescriptors.cols << " " << _slamDescriptors.type());
    ROS_INFO_STREAM("GROUND TRUTH KEYPOINTS= " << _groundTruthKeypoints.size());
    ROS_INFO_STREAM("GROUND TRUTH DESCRIPTORS=" << _groundTruthDescriptors.rows << " "  << _groundTruthDescriptors.cols << " " << _groundTruthDescriptors.type());
    /*ROS_INFO_STREAM("MATCHING RATIO=" << _matchingRatio);*/

   //!< draw Keypoints
   cv::Mat img_keypoints_1, img_keypoints_2;
   cv::drawKeypoints(groundTruthMap, _groundTruthKeypoints, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
   cv::drawKeypoints(slamMap, _slamKeypoints, img_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
   
   if (img_keypoints_1.rows > img_keypoints_2.rows)
   {
     _featuresImage= cv::Mat(img_keypoints_1.rows, img_keypoints_1.cols + img_keypoints_2.cols, img_keypoints_1.type());
     img_keypoints_1.copyTo(_featuresImage(cv::Rect(0, 0, img_keypoints_1.cols, img_keypoints_1.rows)));
     img_keypoints_2.copyTo(_featuresImage(cv::Rect(img_keypoints_1.cols, 0, img_keypoints_2.cols, img_keypoints_2.rows)));
   }

   else
   {
     _featuresImage = cv::Mat(img_keypoints_2.rows, img_keypoints_1.cols + img_keypoints_2.cols, img_keypoints_2.type());
     img_keypoints_2.copyTo(_featuresImage(cv::Rect(0, 0, img_keypoints_2.cols, img_keypoints_2.rows)));
     img_keypoints_1.copyTo(_featuresImage(cv::Rect(img_keypoints_2.cols, 0, img_keypoints_1.cols, img_keypoints_1.rows)));
   }

    if(!params.benchmarking)
    {

      imshow("Keypoints 1", img_keypoints_1);
      imshow("Keypoints 2", img_keypoints_2);
      cv::imwrite("/home/marios/keypoints1.png", img_keypoints_1);
      cv::imwrite("/home/marios/keypoints2.png", img_keypoints_2);
      cv::waitKey(1000);
    }

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
    cv::drawMatches(slamMap, _slamKeypoints, groundTruthMap, _groundTruthKeypoints,
                   filteredMatches, imgmatches1, cv::Scalar::all(-1), cv::Scalar::all(-1));

   _initialMatchedImage = imgmatches1.clone();
    //!< Show detected matches
    if(!params.benchmarking)
    {
      imshow("Initial Matches", imgmatches1);
      cv::waitKey(1000);
      //imwrite("/home/marios/wrongMatches.png", imgmatches1);
    }
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


 /*   ROS_INFO_STREAM("MATCHED KEYPOINTS=" << slamMatchedKeyPoints.size() << " " << groundTruthMatchedKeyPoints.size() << " " << filteredMatches.size());*/
    /*ROS_INFO_STREAM("MATCHED COORDS=" << slamMatchedCoords.size() << " " << groundTruthMatchedCoords.size());*/
       //ROS_INFO_STREAM("MATCHED EVALUATED KEYPOINTS=" << evalfil1.size() << " " << evalfil2.size() << " " << evalmatches.size());

    //ROS_INFO_STREAM("MAX RANSAC REPROJECTION ERROR=" << params.ransacReprjError);
/*for (int i = 0; i < slamMatchedCoords.size(); i++){*/
      //std::cout << "coords " << slamMatchedCoords[i] << " " << groundTruthMatchedCoords[i] << " ";
      //std::cout << "keypoints " << slamMatchedKeyPoints[i].pt << " " << groundTruthMatchedKeyPoints[i].pt << " ";}
        /*std::cout << std::endl;*/

    std::vector<uchar> mask;
    if(slamMatchedCoords.size() < 2)
    {
      ROS_WARN("similarity transform needs at least 2 points to be computed");
      return;
    }
    //double best_error;
    cv::Mat T;

/*    estimateTransform(groundTruthMatchedCoords, slamMatchedCoords,*/
                       //1000, _ransacReprjError, 3, mask, T, best_error);

  /*  int valid = estimateAffine2D(groundTruthMatchedCoords, slamMatchedCoords, T, mask, params.ransacReprjError);*/
    int valid = estimateAffinePartial2D(groundTruthMatchedCoords, slamMatchedCoords, T, mask, params.ransacReprjError);
    if (!valid)
    {
      ROS_WARN("Cannot find transform");
      return;
    }

 /*   std::cout << "ransac result=" << valid << std::endl; */

    /*std::cout << "H = "<< std::endl << " "  << T << std::endl << std::endl;*/

   if(T.empty() || std::count( mask.begin(), mask.end(), 1) < 2)
   {
     ROS_WARN("H contain no data, cannot find valid transformation");
     return;
   }

   //std::cout << " RANSAC inliers = " <<  std::accumulate(mask.begin(), mask.end(), 0) << std::endl;
   //std::vector<cv::Point2f> inliersCoords1, inliersCoords2;
   int inliers = 0;
   for (int i = 0; i < mask.size(); i++)
   {
     if((int)mask[i] == 1)
     {
       inliers++;
       //inliersCoords1.push_back(slamMatchedCoords[i]);
       //inliersCoords2.push_back(groundTruthMatchedCoords[i]);
     }
   }

   _inliers = inliers;
  
   //!< draw matches
   cv::drawMatches( slamMap, _slamKeypoints, groundTruthMap, _groundTruthKeypoints,
                   filteredMatches, imgmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   reinterpret_cast<const std::vector<char>&> (mask), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
   
   _finalMatchedImage = imgmatches.clone();
    ////-- Show detected matches
    if(!params.benchmarking)
    {
      imshow("RANSAC Matches", imgmatches);
      cv::waitKey(1000);
    }

   cv::Mat image(_slamMap.size(), _slamMap.type());
   cv::Mat image1(_slamMap.size(), _slamMap.type(), cv::Scalar(127));
   //image1 = _slamMap;
   //std::cout << "T.type=" << T.type() << std::endl;

   cv::warpAffine(_groundTruthMap, image, T, image.size(), cv::INTER_NEAREST, IPL_BORDER_CONSTANT, cv::Scalar::all(127));
    
  // keep only overlapping slamMap region;
  cv::Size s = _groundTruthMap.size();
  std::vector<cv::Point2f> vertices;
  vertices.push_back(cv::Point2f(0, 0));
  vertices.push_back(cv::Point2f(0, s.height - 1));
  vertices.push_back(cv::Point2f(s.width - 1, s.height - 1));
  vertices.push_back(cv::Point2f(s.width - 1, 0));
  cv::transform(vertices, vertices, T);
  double tWidth = cv::norm(vertices[0] - vertices[3]);
  double tHeight = cv::norm(vertices[0] - vertices[1]);
  //std::cout << tWidth << " " << tHeight << std::endl;
  if(tWidth > 10000 && tHeight > 10000)
  {
    ROS_WARN("Too large transformed image");
    return;
  }


/*  for (int i =0; i < vertices.size(); i++)*/
  /*std::cout << vertices[i] << std::endl;*/
  cv::Mat maskImage = cv::Mat(image.size(), CV_8U, cv::Scalar(0));
 
  std::vector< std::vector<cv::Point> >  co_ordinates;
  co_ordinates.push_back(std::vector<cv::Point>());
  co_ordinates[0].push_back(vertices[0]);
  co_ordinates[0].push_back(vertices[1]);
  co_ordinates[0].push_back(vertices[2]);
  co_ordinates[0].push_back(vertices[3]);
  cv::drawContours(maskImage,co_ordinates, 0, cv::Scalar(255),CV_FILLED, 8);
  int nonzero = cv::countNonZero(image);
  double  obstaclePercentage = 1.0 - (double) nonzero/image.total();
  if(cv::countNonZero(maskImage) == 0 || nonzero == 0 || obstaclePercentage > 0.30)
  {
    ROS_WARN("SOMETHING WENT WRONG");
    return;
  }
  //_slamMap.copyTo(image1,maskImage);                                                

   for (int i = 0; i < image1.rows; i++)
     for (int j = 0; j < image1.cols; j++)
      if(maskImage.at<unsigned char>(i, j) == 255)
        if(image.at<unsigned char>(i, j) == 0 || image.at<unsigned char>(i,j) ==255 )
          image1.at<unsigned char>(i, j) = _slamMap.at<unsigned char>(i, j);
  
  /* double mse1, mse2;*/
   //mse1 = getMSE(image1, image);
   //mse2 = getMSE(_slamMap, image);

   //double overlapError = 1 - (mse1/mse2);

   /*std::cout <<  " overlapError= " << overlapError << std::endl;  */
  
    int agr,dis;
    agreement(image, image1, agr, dis);
    //std::cout << "agr=" << agr << " dis=" <<  dis << std::endl;
    _acceptance = 1.0 - dis / (double)(agr + dis);
    //std::cout << "Acceptance=" << _acceptance << std::endl;
if(!params.benchmarking)
  {
    cv::imshow("GroundTruthMap Transformed", image);
    cv::imshow ("SLamMap Overlapped", image1);
    cv::imshow("MaskImage", maskImage);
    cv::waitKey(1000);
  }
  /*  std::cout << "image1 type " << image1.type() << " " << image1.channels() <<  " " << image1.size() << std::endl;*/
    //std::cout << "image type " << image.type() << " " << image.channels() <<  " " << image.size() << std::endl;

    //!< compute OMSE metric 
    _omseMetric.reset(new OmseMetric(image, image1));
    params.closestPointMethod = "Brushfire";
    _omseMetric->calculateMetric(params);
    double omse1 = _omseMetric->getResult();
    //std::cout << omse1 << std::endl;
   if(omse1 == std::numeric_limits<double>::max())
  {
    ROS_WARN("SOMETHING WENT WRONG");
    return;
  }


/*    _omseMetric.reset(new OmseMetric(image, _slamMap));*/
    //params.closestPointMethod = "Brushfire";
    //_omseMetric->calculateMetric(params);
    //double omse2 = _omseMetric->getResult();

    //_omseMetric.reset(new OmseMetric(image1, image));
    //params.closestPointMethod = "Brushfire";
    //_omseMetric->calculateMetric(params);
    //double omse3 = _omseMetric->getResult();

    _quality = std::exp(-(3/std::sqrt(tWidth*tWidth+tHeight*tHeight)* omse1));
    //double quality2 = std::exp(-(3/std::sqrt(tWidth*tWidth+tHeight*tHeight)* omse3));
    //std::cout << "OMSE=" << omse2 << std::endl;
    //std::cout << "only overlap region OMSE(map1-map2)=" << omse1 << std::endl;
    //std::cout << "only overlap region OMSE(map2-map1)=" << omse3 << std::endl;
    //std::cout << "quality1=  " << _quality << std::endl;
    //std::cout << "quality2=  " << quality2 << std::endl;
    //double overlapOMSE = 1 - (omse1 / omse2);

    // stich,merge blend images computing optimal size
    cv::Mat mergedImage = merge_images(T, vertices);

    cv::Mat usefulMergedImage = cv::Mat(image.size(), image.type());
    addWeighted(image, .5, image1, .5, 0.0, usefulMergedImage);
 /*   cv::imshow("usefulMergedImage", usefulMergedImage);*/
    /*cv::waitKey(1000);*/
    double overlapped = 0;
    double overlapped1 = 0;
    double overlapped2 = 0;

    for (int i = 0; i < usefulMergedImage.rows; i++)
      for (int j = 0; j < usefulMergedImage.cols; j++)
      {
        if(usefulMergedImage.at<uchar>(i,j) == 0 || usefulMergedImage.at<uchar>(i,j) == 255)
          overlapped++;
      }

    for (int i = 0; i < _groundTruthMap.rows; i++)
      for (int j = 0; j < _groundTruthMap.cols; j++)
      {
        if(_groundTruthMap.at<uchar>(i,j) == 0 || _groundTruthMap.at<uchar>(i,j) == 255)
          overlapped1++;
      }

    for (int i = 0; i < _slamMap.rows; i++)
      for (int j = 0; j < _slamMap.cols; j++)
      {
        if(_slamMap.at<uchar>(i,j) == 0 || _slamMap.at<uchar>(i,j) == 255)
          overlapped2++;
      }

       _overlap = overlapped / (overlapped1 + overlapped2 -overlapped) * 100;
    //std::cout << " Overlapping Area = " <<  _overlap << std::endl;

    _mergedImage = mergedImage.clone();
    if(!params.benchmarking)
    {
      cv::imshow("MergedImage", mergedImage);
      cv::waitKey(1000);
    }

    //std::cout << mask.size() << std::endl;
  /*  int counter = 0;*/
    //for( int i = 0; i < filteredMatches.size(); i++ )
    //{
      ////std::cout << (int)mask[i]<<std::endl;
      //if( int(mask[i]) == 1)
      //{
 //[>     ROS_INFO( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  --Distance %f  \n", i, filteredMatches[i].queryIdx, filteredMatches[i].trainIdx, filteredMatches[i].distance );<]
      //[>counter++;<]
      //}
    /*}*/

    //std::cout << counter << std::endl;
    _result = omse1;//_omseMetric->getResult();
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

double FeatureMetrics::getMSE(const cv::Mat& image1, const cv::Mat& image)
{
    cv::Mat m1, m2, s1;
    image1.copyTo(m1);
    image.copyTo(m2);
    cv::absdiff(m1, m2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    cv::Scalar sum = cv::sum(s1);        // sum elements per channel

    double sse = sum.val[0] + sum.val[1] + sum.val[2]; // sum channels

    double mse  = sse / (double)(m1.channels() * m1.total());

    printf("mse = %f", mse);  

    return mse;
}

void FeatureMetrics::agreement(const cv::Mat& image, const cv::Mat&image1, int& agr, int& dis)
{
  int a,d;
  a = d = 0;
  for(int i = 0; i < image.rows; i++)
    for(int j  = 0; j < image.cols; j++)
    {
      if(image.at<uchar>(i,j) == 255 && image1.at<uchar>(i,j) == 255)
      {
        a++;
      }
  
      if(image.at<uchar>(i,j) == 0 && image1.at<uchar>(i,j) == 0)
      {
        a++;
      }

      if(image.at<uchar>(i,j) == 0 && image1.at<uchar>(i,j) == 255)
      {
        d++;
      }
  
      if(image.at<uchar>(i,j) == 255 && image1.at<uchar>(i,j) == 0)
      {
        d++;
      }
      agr = a;
      dis = d;

    }

}

cv::Mat FeatureMetrics::merge_images(const cv::Mat& T,std::vector<cv::Point2f> vertices)
{
  double offsetX = 0.0;
  double offsetY = 0.0;
  //Get max offset outside of the image
  for(size_t i = 0; i < 4; i++) {
    if(vertices[i].x < offsetX) {
      offsetX = vertices[i].x;
    }

    if(vertices[i].y < offsetY) {
      offsetY = vertices[i].y;
    }
  }

  offsetX = -offsetX;
  offsetY = -offsetY;
  //std::cout << "offsetX=" << offsetX << " ; offsetY=" << offsetY << std::endl;

  //Get max width and height for the new size of the panorama
  double maxX = std::max((double) _slamMap.cols+offsetX, (double) std::max(vertices[2].x, vertices[3].x)+offsetX);
  double maxY = std::max((double) _slamMap.rows+offsetY, (double) std::max(vertices[1].y, vertices[3].y)+offsetY);
  //std::cout << "maxX=" << maxX << " ; maxY=" << maxY << std::endl;

  cv::Size size_warp(maxX, maxY);
  cv::Mat panorama(size_warp, _slamMap.type());
  cv::Mat panorama1(size_warp, _slamMap.type(), cv::Scalar(127));


  //Create the transformation matrix to be able to have all the pixels
  //cv::Mat H2 = cv::Mat::eye(2, 3, CV_64F);
  cv::Mat T1= cv::Mat(2,3, T.type());
  T.copyTo(T1);
  T1.at<double>(0,2) = T1.at<double>(0,2) + offsetX;
  T1.at<double>(1,2) = T1.at<double>(1,2) + offsetY;

  cv::warpAffine(_groundTruthMap, panorama, T1, size_warp, cv::INTER_NEAREST, IPL_BORDER_CONSTANT, cv::Scalar::all(127));

  for (int i = 0; i < _slamMap.rows; i++)
    for( int j = 0; j < _slamMap.cols; j++)
      panorama1.at<uchar>(i+offsetY, j+offsetX) = _slamMap.at<uchar>(i,j);
/*  if(!params.benchmarking)*/
  //{
    //cv::imshow("panorama", panorama);
    //cv::imshow("panorama1", panorama1);
    //cv::waitKey(1000);
  /*}*/
  addWeighted(panorama1, .5, panorama, .5, 0.0, panorama1);

  return panorama1;
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
  sensor_msgs::Image FeatureMetrics::getFeaturesImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _featuresImage;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.toImageMsg(temp);
    return temp;
  }

  sensor_msgs::Image FeatureMetrics::getInitialMatchedImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _initialMatchedImage;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //std::cout << _initialMatchedImage.type() << " " << _initialMatchedImage.channels() << std::endl;
    out_msg.toImageMsg(temp);
    return temp;
  }
 
   sensor_msgs::Image FeatureMetrics::getFinalMatchedImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _finalMatchedImage;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //std::cout << _finalMatchedImage.type() << " " << _finalMatchedImage.channels() << std::endl;
    out_msg.toImageMsg(temp);
    return temp;
  }

  sensor_msgs::Image FeatureMetrics::getMergedImage()
  {
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image temp;
    out_msg.image = _mergedImage;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    //std::cout << _mergedImage.type() << " " << _mergedImage.channels() << std::endl;
    out_msg.toImageMsg(temp);
    return temp;
  }
}
