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
#include "ogm_evaluation/ogm_evaluation_utils/maps_alignment.h"

namespace ogm_evaluation
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
  void Alignment::alignMaps(const ogm_msgs::MapPose& _transform,
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

    //cv::absdiff(_groundTruthMap, _slamMap, diff);

    ROS_INFO_STREAM("GROUND TRUTH MAP SIZE after padding=" << _groundTruthMap.size());
    ROS_INFO_STREAM("SLAM MAP SIZE after padding =" << _slamMap.size());
    ROS_INFO_STREAM("X=" << _transform.pose.x);
    ROS_INFO_STREAM("Y=" << _transform.pose.y);
    ROS_INFO_STREAM("theta=" << _transform.pose.theta);
    ROS_INFO_STREAM("scale=" << _transform.scale);
    ROS_INFO_STREAM("SlamScale=" << _transform.slamOffsetScale);
    ROS_INFO_STREAM("GroundScale=" << _transform.groundTruthOffsetScale);
  }

}  // namespace ogm_evaluation
