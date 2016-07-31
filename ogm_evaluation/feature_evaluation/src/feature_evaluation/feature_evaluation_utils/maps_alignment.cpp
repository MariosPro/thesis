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
#include "feature_evaluation/feature_evaluation_utils/maps_alignment.h"

namespace feature_evaluation
{
  /**
  @class Alignment
  @brief Implements the OGM alignment methods
  **/ 
  Alignment::Alignment()
  {

  }

  /**
  @brief Method that aligns the two maps using the transform received from server
  @return void
  **/
  void Alignment::alignMaps(const ogm_communications::MapPose& _transform,
                       cv::Mat& _groundTruthMap,
                       cv::Mat& _slamMap)
  {
    cv::Mat transformMat;

    // resize ground truth map to offset scale
    cv::resize(_groundTruthMap, _groundTruthMap, cv::Size(), _transform.groundTruthOffsetScale, _transform.groundTruthOffsetScale, cv::INTER_NEAREST);

    // construct the transformation Matrix
    cv::Point2f center(_groundTruthMap.cols/2.0,  _groundTruthMap.rows/2.0);
    transformMat = cv::getRotationMatrix2D(center, -_transform.pose.theta, _transform.scale);

    transformMat.at<double>(0,2) += _transform.pose.x;
    transformMat.at<double>(1,2) += _transform.pose.y;

    // apply the transformation
    cv::warpAffine(_groundTruthMap, _groundTruthMap, transformMat, _groundTruthMap.size(), cv::INTER_NEAREST, IPL_BORDER_CONSTANT, cv::Scalar::all(127));

    // resize slam produced map to offset scale
    cv::resize(_slamMap, _slamMap, cv::Size(), _transform.slamOffsetScale,  _transform.slamOffsetScale, cv::INTER_NEAREST);

    ROS_INFO_STREAM("GROUND TRUTH MAP SIZE after scaling=" << _groundTruthMap.size());
    ROS_INFO_STREAM("SLAM MAP SIZE after scaling =" << _slamMap.size());

    //"unknown" padding to fix images size
    int paddedWidth, paddedHeight;
    paddedHeight = abs(_slamMap.rows - _groundTruthMap.rows);
    paddedWidth = abs(_slamMap.cols - _groundTruthMap.cols);

    ROS_INFO_STREAM("PADDED WIDTH/HEIGHT=" << paddedWidth << " " << paddedHeight);

    if(_slamMap.rows >= _groundTruthMap.rows)
      cv::copyMakeBorder(_groundTruthMap, _groundTruthMap, 0, paddedHeight, 0, 0, IPL_BORDER_CONSTANT, cv::Scalar(127));
    else
      cv::copyMakeBorder(_slamMap, _slamMap, 0, paddedHeight, 0, 0, IPL_BORDER_CONSTANT, cv::Scalar(127));

    if(_slamMap.cols >= _groundTruthMap.cols)
      cv::copyMakeBorder(_groundTruthMap, _groundTruthMap, 0, 0, 0, paddedWidth, IPL_BORDER_CONSTANT, cv::Scalar(127));
    else
      cv::copyMakeBorder(_slamMap, _slamMap, 0, 0, 0, paddedWidth, IPL_BORDER_CONSTANT, cv::Scalar(127));

    ROS_INFO_STREAM("GROUND TRUTH MAP SIZE after padding=" << _groundTruthMap.size());
    ROS_INFO_STREAM("SLAM MAP SIZE after padding =" << _slamMap.size());
    ROS_INFO_STREAM("X=" << _transform.pose.x);
    ROS_INFO_STREAM("Y=" << _transform.pose.y);
    ROS_INFO_STREAM("theta=" << _transform.pose.theta);
    ROS_INFO_STREAM("scale=" << _transform.scale);
    ROS_INFO_STREAM("SlamScale=" << _transform.slamOffsetScale);
    ROS_INFO_STREAM("GroundScale=" << _transform.groundTruthOffsetScale);
  }

  void Alignment::ICP(const cv::Mat& target, const cv::Mat& reference, int iterations)
  {
    std::vector<cv::Point> refPoints, targetPoints;
    cv::Mat refPointsMat, newRefPointsMat, targetPointsMat;
    cv::Mat transform = cv::Mat::eye(2, 3, CV_32FC1);

    //!< keep only obstacle points (1 point per row)
    for(int i = 0; i < target.rows; i++)
      for(int j = 0; j < target.cols; j++)
        if(target.at<uchar>(i, j) == 0)
          targetPoints.push_back(cv::Point(i, j));
 
    for(int i = 0; i < reference.rows; i++)
      for(int j = 0; j < reference.cols; j++)
        if(reference.at<uchar>(i, j) == 0)
          refPoints.push_back(cv::Point(i, j));

    refPointsMat = cv::Mat(refPoints);
    targetPointsMat = cv::Mat(targetPoints);
    targetPointsMat.convertTo(targetPointsMat, CV_32FC1);
    refPointsMat.convertTo(refPointsMat, CV_32FC1);
    targetPointsMat = targetPointsMat.reshape(1);
    refPointsMat = refPointsMat.reshape(1);
    double error = RMSE(targetPointsMat, refPointsMat);

     std::cout << "RMSE=" << error << std::endl;

    std::cout << "refPointsMat=" << refPointsMat.size() << refPointsMat.type() << std::endl;
    std::cout << "targetPointsMat=" << targetPointsMat.size() << targetPointsMat.type() << std::endl;

    cv::Mat temp = (cv::Mat_<float>(1, 3) << 0, 0, 1);
    cv::Mat transformHom = cv::Mat(3, 3, CV_32FC1);
       while (iterations > 0)
    {
      cv::Mat lastTransform, newTransform, closestPointsMat;
      lastTransform = transform;
      cv::Mat lastTransformHom = cv::Mat(0, 3, CV_32FC1);
      cv::Mat newTransformHom = cv::Mat(0, 3, CV_32FC1);


      lastTransformHom.push_back(transform.row(0));
      lastTransformHom.push_back(transform.row(1));
      lastTransformHom.push_back(temp);

      std::cout << "lastTransformHom= "<< std::endl << " "  << lastTransformHom << std::endl << std::endl;
      match(targetPointsMat, refPointsMat, closestPointsMat);

      //closestPointsMat.copyTo(refPointsMat);
      findTransform(targetPointsMat, closestPointsMat, newTransform);
     
      newTransformHom.push_back(newTransform.row(0));
      newTransformHom.push_back(newTransform.row(1));
      newTransformHom.push_back(temp);

      std::cout << "newTransformHom= "<< std::endl << " "  << newTransformHom << std::endl << std::endl;
      transformHom =  newTransformHom * lastTransformHom;
     /* transform.release();*/
      /*transform = cv::Mat(2, 3, CV_32FC1);*/
      for(int i = 0; i < transformHom.rows-1; i++)
        transformHom.row(i).copyTo(transform.row(i));

      std::cout << "transformHom= "<< std::endl << " "  << transformHom << std::endl << std::endl;
      newRefPointsMat = cv::Mat(0, refPointsMat.cols, refPointsMat.type());
      //cv::warpAffine(refPointsMat, newRefPointsMat, transform, refPointsMat.size());
      applyTransformation(refPointsMat, newRefPointsMat, transform);
      cv::Mat transformedclosestPointsMat = cv::Mat(0, closestPointsMat.cols, closestPointsMat.type());
      applyTransformation(closestPointsMat, transformedclosestPointsMat, transform);
      double error = RMSE(targetPointsMat, transformedclosestPointsMat);

      std::cout << "RMSE=" << error << std::endl;
      //!< check for termination.
      if (error < 0.0001) 
        break;

      refPointsMat.release();
      newRefPointsMat.copyTo(refPointsMat);

      std::cout << "refPointsMat=" << refPointsMat.size() << refPointsMat.type() << std::endl;
      std::cout << "transformedclosestPointsMat=" << transformedclosestPointsMat.size() << transformedclosestPointsMat.type() << std::endl;

      //std::cout << "newRefPointsMat = "<< std::endl << " "  << newRefPointsMat << std::endl << std::endl;

      displayMap(refPointsMat, reference.size());
      
      iterations--;
    }
  }
  
  void Alignment::match(const cv::Mat& targetPointsMat, const cv::Mat& refPointsMat, cv::Mat& closestPointsMat)
  {
    //!< find nearest neighbors using FLANN
    cv::Mat m_indices(targetPointsMat.rows, 1, CV_32S);
    cv::Mat m_dists(targetPointsMat.rows, 1, CV_32F);
    std::vector<cv::Point> closestPoints;
      //std::cout << "refPointsMat = "<< std::endl << " "  << refPointsMat << std::endl << std::endl;
      //std::cout << "targetPointsMat = "<< std::endl << " "  << targetPointsMat << std::endl << std::endl;
    cv::flann::Index flann_index(refPointsMat, cv::flann::KDTreeIndexParams(2)); // using 2 randomized kdtrees
    flann_index.knnSearch(targetPointsMat, m_indices, m_dists, 1, cv::flann::SearchParams(64)); 
    std::vector<int> pointPairs;
    int* indices_ptr = m_indices.ptr<int>(0);
    for (int i = 0; i < targetPointsMat.rows; i++)
    {
      pointPairs.push_back(indices_ptr[i]);
      cv::Point p = cv::Point(refPointsMat.row(indices_ptr[i])); //refPointsMat.at<cv::Point>(pointPairs[i], 0);
      closestPoints.push_back(p);
   /*   std::cout << closestPoints[i] << " ";*/
      //std::cout << indices_ptr[i] << " ";
    }
    //!< constuct closestPoints Matrix
    closestPointsMat = cv::Mat(closestPoints);
    closestPointsMat = closestPointsMat.reshape(1);
    closestPointsMat.convertTo(closestPointsMat, CV_32FC1);
    std::cout << "closestPointsMat=" << closestPointsMat.size() << closestPointsMat.type() << std::endl;
    //std::cout << "closestPointsMat = "<< std::endl << " "  << closestPointsMat << std::endl << std::endl;

  }

  void Alignment::findTransform(cv::Mat targetPointsMat, cv::Mat& closestPointsMat, cv::Mat& transform)
  {
    cv::Mat translatedTargetPointsMat = cv::Mat(targetPointsMat.size(), targetPointsMat.type());
    //!< extract centroids of correspondences
    cv::Mat centroid1 = cv::Mat(2, 1, closestPointsMat.type());
    cv::Mat centroid2 = cv::Mat(2, 1, closestPointsMat.type());
    for (int i = 0; i < closestPointsMat.cols; i++)
    {
      centroid1.at<float>(i) = cv::mean(closestPointsMat.col(i)).val[0];
      centroid2.at<float>(i) = cv::mean(targetPointsMat.col(i)).val[0];
    }

    std::cout << "centroid1=" << std::endl << " "  << centroid1 << std::endl << std::endl;
    std::cout << "centroid2= " << std::endl << " "  << centroid2 << std::endl << std::endl;

    //!< substract mean from points
    for (int i = 0; i < closestPointsMat.rows; i++)
    {
        closestPointsMat.at<float>(i, 0) = closestPointsMat.at<float>(i, 0) - centroid1.at<float>(0);//centroid1[0].val[0];
        closestPointsMat.at<float>(i, 1) = closestPointsMat.at<float>(i, 1) - centroid1.at<float>(1);//centroid1[1].val[0];
    }
    for (int i = 0; i < closestPointsMat.rows; i++)
    {
        translatedTargetPointsMat.at<float>(i, 0) = targetPointsMat.at<float>(i, 0) - centroid2.at<float>(0);//centroid2[0].val[0];
        translatedTargetPointsMat.at<float>(i, 1) = targetPointsMat.at<float>(i, 1) - centroid2.at<float>(1);//centroid2[1].val[0];
    }

    //std::cout << "translatedTargetPointsMat = "<< std::endl << " "  << translatedTargetPointsMat << std::endl << std::endl;
    //std::cout << "closestPointsMat = "<< std::endl << " "  << closestPointsMat << std::endl << std::endl;

    //!< perform Singular Value Decomposition 
    cv::Mat H(2, 2, CV_32FC1);
    for(int i = 0; i < translatedTargetPointsMat.rows - 1; i++)
    {
      cv::Mat mci = translatedTargetPointsMat(cv::Range(i, i+1), cv::Range(0, 2));
      cv::Mat dci = closestPointsMat(cv::Range(i, i+1), cv::Range(0, 2));
      H = H + mci.t() * dci;
    /*  std::cout << "mci = "  << std::endl << " " << mci << std::endl;*/
      /*std::cout << "dci = "  << std::endl << " " << dci << std::endl;*/
    }

    std::cout << "H = "  << std::endl << " " << H << std::endl;
    cv::SVD svd(H);
    cv::Mat R = svd.vt.t() * svd.u.t();
    cv::Mat T;
    double det_R = cv::determinant(R);
    if (det_R < 0)
    {
      cv::Mat I = - cv::Mat::eye(2, 2, R.type());
      std::cout << "I = "  << std::endl << " " << I << std::endl;
      R = svd.vt.t() * I * svd.u.t();
    }
    T = centroid1 - (R * centroid2);

    std::cout << "R = "  << std::endl << " " << R << std::endl;
    std::cout << "T = "  << std::endl << " " << T << std::endl;

    cv::hconcat(R, T, transform);
    std::cout << "Transform = "  << std::endl << " " << transform << std::endl;
  }

  void Alignment::applyTransformation(const cv::Mat& refPointsMat, cv::Mat& newRefPointsMat, const cv::Mat& transform)
  { cv::Mat translation, rotation, transPointsMat;
    translation = cv::Mat(2, 1, transform.type());
    rotation = cv::Mat(2, 2, transform.type());
    transPointsMat = cv::Mat(0, refPointsMat.cols, refPointsMat.type());
    translation = transform.col(2);
    cv::transpose(translation, translation);
    transform.col(0).copyTo(rotation.col(0));
    transform.col(1).copyTo(rotation.col(1));
    std::cout << "Translation = "  << std::endl << " " << translation << std::endl;
    std::cout << "Rotation = "  << std::endl << " " << rotation << std::endl;
    for(int i = 0; i < refPointsMat.rows; i++)
    {
      cv::Mat temp = refPointsMat.row(i)+ translation;
      transPointsMat.push_back(temp);
      cv::Mat temp1 = transPointsMat.row(i) * rotation;
      newRefPointsMat.push_back(temp1);
    }
    //std::cout << "transPointsMat=" << std::endl << " " << transPointsMat << std::endl;
    //std::cout << "newRefPointsMat=" << std::endl << " " << newRefPointsMat << std::endl;
  }

  double Alignment::RMSE(const cv::Mat& targetPointsMat, const cv::Mat& refPointsMat)
  {
    if(targetPointsMat.size() != refPointsMat.size())
    {
      ROS_ERROR_STREAM("points size mismatch");
      return 0.0;
    }
    //!< first take difference of the two arrays
    cv::Mat diff = cv::Mat(targetPointsMat.size(), CV_64FC1);
    diff = targetPointsMat - refPointsMat;
    //!< square the differences
    cv::Mat squares = cv::Mat(diff.size(), CV_64FC1);
    for (int i = 0; i < diff.rows; i++)
      for (int j = 0; j < diff.cols; j++)
        squares.at<double>(i, j) = cv::pow(diff.at<double>(i, j), 2);
    //!< vector for the sum of the rows
    std::vector<double> sqrtsum;
    for (int i =0; i < diff.rows; i++)
      sqrtsum.push_back(cv::sum(squares.row(i)).val[0]);
    
    //!<get the square roots. 
    std::vector<double> sqrts;
    for (int q = 0; q < sqrtsum.size(); q++) {
            sqrts.push_back(std::sqrt(sqrtsum.at(q)));
    }

    //!< calculate rmse as the sum of square roots. 
    double rmse = 0.0;
    for (int x = 0; x < sqrts.size(); x++) {
            rmse += sqrts.at(x);;
    }
    return rmse;
  }

  void Alignment::displayMap(cv::Mat refPointsMat, cv::Size size)
  {

    //refPointsMat.convertTo(refPointsMat, CV_8UC1);
    cv::Mat display = cv::Mat(size, CV_8UC1);
    for(int i = 0; i < size.height; i++)
      for (int j = 0; j < size.width; j++)
         display.at<uchar>(i, j) = 255;
    for (int i = 0; i < refPointsMat.rows; i++)
        if(refPointsMat.at<float>(i, 0) > 0 && refPointsMat.at<float>(i, 0) < display.rows &&
            refPointsMat.at<float>(i, 1) > 0 && refPointsMat.at<float>(i, 1) < display.rows)
        display.at<uchar>(refPointsMat.at<float>(i, 0), refPointsMat.at<float>(i, 1)) = 0;
    
    cv::imshow("transformedImage", display);
    cv::waitKey(0);
  }
}  // namespace feature_evaluation
