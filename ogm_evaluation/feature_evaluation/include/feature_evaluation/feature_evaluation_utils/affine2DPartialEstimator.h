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
#ifndef AFFINE2DPARTIALESTIMATOR_H
#define AFFINE2DPARTIALESTIMATOR_H

#include <limits>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

/**
@namespace feature_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace feature_evaluation
{

class Affine2DPartialEstimator
{
public:
  Affine2DPartialEstimator(double _threshold, double _confidence, int _modelPoints, int _maxIters);
  
  int runKernel(cv::InputArray _m1, cv::InputArray _m2, 
                cv::OutputArray _model);

    bool runRANSAC(cv::InputArray _m1, cv::InputArray _m2, 
                   cv::OutputArray _model, cv::OutputArray _mask);
   
    bool getSubset(const cv::Mat& m1, const cv::Mat& m2,
                   cv::Mat& ms1, cv::Mat& ms2, cv::RNG& rng,
                   int maxAttempts);
    
    bool checkSubset(cv::InputArray _ms1, cv::InputArray _ms2, int count);
    
    int findInliers(const cv::Mat& m1, const cv::Mat& m2, 
                    const cv::Mat& model, cv::Mat& err,
                    cv::Mat& mask, double thresh, float* totalError);

    void computeError(cv::InputArray _m1, cv::InputArray _m2, 
                    cv::InputArray _model, cv::OutputArray _err); 
    
    int RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters);

protected:
  CvRNG rng;
  int modelPoints;
  //CvSize modelSize;
  //int maxBasicSolutions;
  bool checkPartialSubsets;
  double threshold;
  double confidence;
  int maxIters;

};

int estimateAffinePartial2D(cv::InputArray _from, cv::InputArray _to,
  cv::OutputArray _out, cv::OutputArray _inliers,
  double param1 = 3, double param2 = 0.99);
}
#endif
