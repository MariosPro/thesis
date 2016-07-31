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
#ifndef AFFINE2DESTIMATOR_H
#define AFFINE2DESTIMATOR_H

#include <limits>
#include <opencv2/opencv.hpp>

/**
@namespace feature_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace feature_evaluation
{

class Affine2DEstimator
{
public:
  Affine2DEstimator();
  int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 
  bool runRANSAC( const CvMat* m1, const CvMat* m2, CvMat* model,
          CvMat* mask, double threshold,
          double confidence=0.99, int maxIters=2000 );
  bool getSubset( const CvMat* m1, const CvMat* m2,
          CvMat* ms1, CvMat* ms2, int maxAttempts=1000 );
  bool checkSubset( const CvMat* ms1, int count );
  int findInliers( const CvMat* m1, const CvMat* m2,
          const CvMat* model, CvMat* error,
          CvMat* mask, double threshold );
  void computeReprojError( const CvMat* m1, const CvMat* m2, const CvMat* model, CvMat* error ); 
protected:
  CvRNG rng;
  int modelPoints;
  CvSize modelSize;
  int maxBasicSolutions;
  bool checkPartialSubsets;
};



int estimateAffine2D(cv::InputArray _from, cv::InputArray _to,
  cv::OutputArray _out, cv::OutputArray _inliers,
  double param1=3, double param2=0.99);
}
#endif
