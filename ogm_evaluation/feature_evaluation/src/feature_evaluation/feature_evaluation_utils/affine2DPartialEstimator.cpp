/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "feature_evaluation/feature_evaluation_utils/affine2DPartialEstimator.h"

#include <algorithm>
#include <iterator>
#include <limits>

namespace feature_evaluation
{
  Affine2DPartialEstimator::Affine2DPartialEstimator(double _threshold, double _confidence, int _modelPoints, int _maxIters)
  {
    checkPartialSubsets = false;
    modelPoints= _modelPoints;
    threshold= _threshold;
    confidence= _confidence;
    maxIters= _maxIters;

  }
int Affine2DPartialEstimator::RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters)
{
    if( modelPoints <= 0 )
        ROS_ERROR("the number of model points should be positive");

    p = MAX(p, 0.);
    p = MIN(p, 1.);
    ep = MAX(ep, 0.);
    ep = MIN(ep, 1.);

    // avoid inf's & nan's
    double num = MAX(1. - p, DBL_MIN);
    double denom = 1. - std::pow(1. - ep, modelPoints);
    if( denom < DBL_MIN )
        return 0;

    num = std::log(num);
    denom = std::log(denom);

    return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
}
 
    int Affine2DPartialEstimator::findInliers(const cv::Mat& m1, const cv::Mat& m2, 
                    const cv::Mat& model, cv::Mat& err,
                    cv::Mat& mask, double thresh)

    {
        computeError(m1, m2, model, err);
        mask.create(err.size(), CV_8U);

        CV_Assert( err.isContinuous() && err.type() == CV_32F && mask.isContinuous() && mask.type() == CV_8U);
        const float* errptr = err.ptr<float>();
        uchar* maskptr = mask.ptr<uchar>();
        float t = (float)(thresh*thresh);
        int i, n = (int)err.total(), nz = 0;
        for( i = 0; i < n; i++ )
        {
            int f = errptr[i] <= t;
            maskptr[i] = (uchar)f;
            nz += f;
        }
        return nz;
    }

    bool Affine2DPartialEstimator::getSubset(const cv::Mat& m1, const cv::Mat& m2,
                   cv::Mat& ms1, cv::Mat& ms2, cv::RNG& rng,
                   int maxAttempts=1000) 
    {
        cv::AutoBuffer<int> _idx(modelPoints);
        int* idx = _idx;
        int i = 0, j, k, iters = 0;
        int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
        int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
        int esz1 = (int)m1.elemSize1()*d1, esz2 = (int)m2.elemSize1()*d2;
        int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
        const int *m1ptr = m1.ptr<int>(), *m2ptr = m2.ptr<int>();

        ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
        ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

        int *ms1ptr = ms1.ptr<int>(), *ms2ptr = ms2.ptr<int>();

        CV_Assert( count >= modelPoints && count == count2 );
        CV_Assert( (esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0 );
        esz1 /= sizeof(int);
        esz2 /= sizeof(int);

        for(; iters < maxAttempts; iters++)
        {
            for( i = 0; i < modelPoints && iters < maxAttempts; )
            {
                int idx_i = 0;
                for(;;)
                {
                    idx_i = idx[i] = rng.uniform(0, count);
                    for( j = 0; j < i; j++ )
                        if( idx_i == idx[j] )
                            break;
                    if( j == i )
                        break;
                }
                for( k = 0; k < esz1; k++ )
                    ms1ptr[i*esz1 + k] = m1ptr[idx_i*esz1 + k];
                for( k = 0; k < esz2; k++ )
                    ms2ptr[i*esz2 + k] = m2ptr[idx_i*esz2 + k];
                if( checkPartialSubsets && !checkSubset( ms1, ms2, i+1 ))
                {
                    // we may have selected some bad points;
                    // so, let's remove some of them randomly
                    i = rng.uniform(0, i+1);
                    iters++;
                    continue;
                }
                i++;
            }
            if( !checkPartialSubsets && i == modelPoints && !checkSubset(ms1, ms2, i))
                continue;
            break;
        }

        return i == modelPoints && iters < maxAttempts;
    }
   bool Affine2DPartialEstimator::runRANSAC(cv::InputArray _m1, cv::InputArray _m2, 
                                            cv::OutputArray _model, cv::OutputArray _mask) 
    {
        bool result = false;
        cv::Mat m1 = _m1.getMat(), m2 = _m2.getMat();
        cv::Mat err, mask, model, bestModel, ms1, ms2;

        int iter, niters = MAX(maxIters, 1);
        int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
        int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
        int count = m1.checkVector(d1), count2 = m2.checkVector(d2), maxGoodCount = 0;

        cv::RNG rng((uint64)-1);

        CV_Assert( confidence > 0 && confidence < 1 );
        CV_Assert( count >= 0 && count2 == count );
        if( count < modelPoints )
            return false;

        cv::Mat bestMask0, bestMask;

        if( _mask.needed() )
        {
            _mask.create(count, 1, CV_8U, -1, true);
            bestMask0 = bestMask = _mask.getMat();
            CV_Assert( (bestMask.cols == 1 || bestMask.rows == 1) && (int)bestMask.total() == count );
        }
        else
        {
            bestMask.create(count, 1, CV_8U);
            bestMask0 = bestMask;
        }

        if( count == modelPoints )
        {
            if( runKernel(m1, m2, bestModel) <= 0 )
                return false;
            bestModel.copyTo(_model);
            bestMask.setTo(cv::Scalar::all(1));
            return true;
        }

        for( iter = 0; iter < niters; iter++ )
        {
            int i, nmodels;
            if( count > modelPoints )
            {
                bool found = getSubset( m1, m2, ms1, ms2, rng, 10000 );
                if( !found )
                {
                    if( iter == 0 )
                        return false;
                    break;
                }
            }

            nmodels = runKernel( ms1, ms2, model );
            if( nmodels <= 0 )
                continue;
            CV_Assert( model.rows % nmodels == 0 );
            cv::Size modelSize(model.cols, model.rows/nmodels);

            for( i = 0; i < nmodels; i++ )
            {
                cv::Mat model_i = model.rowRange( i*modelSize.height, (i+1)*modelSize.height );
                int goodCount = findInliers( m1, m2, model_i, err, mask, threshold );

                if( goodCount > MAX(maxGoodCount, modelPoints-1) )
                {
                    std::swap(mask, bestMask);
                    model_i.copyTo(bestModel);
                    maxGoodCount = goodCount;
                    niters = RANSACUpdateNumIters( confidence, (double)(count - goodCount)/count, modelPoints, niters );
                }
            }
        }

        if( maxGoodCount > 0 )
        {
            if( bestMask.data != bestMask0.data )
            {
                if( bestMask.size() == bestMask0.size() )
                    bestMask.copyTo(bestMask0);
                else
                    transpose(bestMask, bestMask0);
            }
            bestModel.copyTo(_model);
            result = true;
        }
        else
            _model.release();

        return result;
    }
  void Affine2DPartialEstimator::computeError(cv::InputArray _m1, cv::InputArray _m2, 
                    cv::InputArray _model, cv::OutputArray _err) 
  {
    cv::Mat m1 = _m1.getMat(), m2 = _m2.getMat(), model = _model.getMat();
    const cv::Point2f* from = m1.ptr<cv::Point2f>();
    const cv::Point2f* to   = m2.ptr<cv::Point2f>();
    const double* F = model.ptr<double>();

    int count = m1.checkVector(2);
    CV_Assert( count > 0 );

    _err.create(count, 1, CV_32F);
    cv::Mat err = _err.getMat();
    float* errptr = err.ptr<float>();

    for(int i = 0; i < count; i++ )
    {
      const cv::Point2f& f = from[i];
      const cv::Point2f& t = to[i];

      double a = F[0]*f.x + F[1]*f.y + F[2] - t.x;
      double b = F[3]*f.x + F[4]*f.y + F[5] - t.y;

      errptr[i] = (float)(a*a + b*b);
    }
  }

    bool Affine2DPartialEstimator::checkSubset(cv::InputArray _ms1, cv::InputArray _ms2, int count) 
    {
        const float threshold = 0.996f;
        cv::Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();

        for( int inp = 1; inp <= 2; inp++ )
        {
            int j, k, i = count - 1;
            const cv::Mat* msi = inp == 1 ? &ms1 : &ms2;
            const cv::Point2f* ptr = msi->ptr<cv::Point2f>();

            CV_Assert( count <= msi->rows );

            // check that the i-th selected point does not belong
            // to a line connecting some previously selected points
            for(j = 0; j < i; ++j)
            {
                cv::Point2f d1 = ptr[j] - ptr[i];
                float n1 = d1.x*d1.x + d1.y*d1.y;

                for(k = 0; k < j; ++k)
                {
                    cv::Point2f d2 = ptr[k] - ptr[i];
                    float denom = (d2.x*d2.x + d2.y*d2.y)*n1;
                    float num = d1.x*d2.x + d1.y*d2.y;

                    if( num*num > threshold*threshold*denom )
                        return false;
                }
            }
        }
        return true;
    }
int  Affine2DPartialEstimator::runKernel( cv::InputArray _m1, cv::InputArray _m2, 
                                          cv::OutputArray _model ) 
    {
      cv::Mat m1 = _m1.getMat(), m2 = _m2.getMat();
        const cv::Point2f* from = m1.ptr<cv::Point2f>();
        const cv::Point2f* to   = m2.ptr<cv::Point2f>();

        // for partial affine trasform we need only 2 points
        const int N = 4;
        double buf[N*N + N + N + 2*3];
        cv::Mat A(N, N, CV_64F, buf);
        cv::Mat B(N, 1, CV_64F, buf + N*N);
        cv::Mat X(N, 1, CV_64F, buf + N*N + N);
        cv::Mat M(2, 3, CV_64F, buf + N*N + N + N);
        double* Adata = A.ptr<double>();
        double* Bdata = B.ptr<double>();
        double* Xdata = X.ptr<double>();
        double* Mdata = M.ptr<double>();
        A = cv::Scalar::all(0);

        for( int i = 0; i < (N/2); i++ )
        {
            Bdata[i*2] = to[i].x;
            Bdata[i*2+1] = to[i].y;

            double *aptr = Adata + i*2*N;
            aptr[0] = from[i].x;
            aptr[1] = -from[i].y;
            aptr[2] = 1.0;
            aptr[4] = from[i].y;
            aptr[5] = from[i].x;
            aptr[7] = 1.0;
        }

        solve(A, B, X, cv::DECOMP_SVD);

        // set model, rotation part is antisymmetric
        Mdata[0] = Mdata[4] = Xdata[0];
        Mdata[1] = -Xdata[1];
        Mdata[2] = Xdata[2];
        Mdata[3] = Xdata[1];
        Mdata[5] = Xdata[3];
        M.copyTo(_model);
        return 1;
    }
  int estimateAffinePartial2D(cv::InputArray _from, cv::InputArray _to,
                           cv::OutputArray _out, cv::OutputArray _inliers,
                           double param1, double param2)
  {
    cv::Mat from = _from.getMat(), to = _to.getMat();
    int count = from.checkVector(2);

    CV_Assert( count >= 0 && to.checkVector(2) == count );

    cv::Mat dFrom, dTo;
    from.convertTo(dFrom, CV_32F);
    to.convertTo(dTo, CV_32F);
    dFrom = dFrom.reshape(2, count);
    dTo = dTo.reshape(2, count);

    const double epsilon = DBL_EPSILON;
    param1 = param1 <= 0 ? 3 : param1;
    param2 = (param2 < epsilon) ? 0.99 : (param2 > 1 - epsilon) ? 0.99 : param2;

    return Affine2DPartialEstimator(param1, param2, 2, 1000).runRANSAC(dFrom, dTo, _out, _inliers);
  }
}
