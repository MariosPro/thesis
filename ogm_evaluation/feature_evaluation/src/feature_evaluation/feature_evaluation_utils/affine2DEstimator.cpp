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
#include "feature_evaluation/feature_evaluation_utils/affine2DEstimator.h"

namespace feature_evaluation
{

  int Affine2DEstimator::findInliers( const CvMat* m1, const CvMat* m2,
  const CvMat* model, CvMat* _err,
  CvMat* _mask, double threshold )
  {
    int i, count = _err->rows*_err->cols, goodCount = 0;
    const float* err = _err->data.fl;
    uchar* mask = _mask->data.ptr;

    computeReprojError(m1, m2, model, _err);
    threshold *= threshold;
    for(i = 0; i < count; i++)
            goodCount += mask[i] = err[i] <= threshold;
    return goodCount;
  }


  void Affine2DEstimator::computeReprojError(const CvMat* m1, const CvMat* m2, const CvMat* model, CvMat* error)
  {
    int count = m1->rows * m1->cols;
    const CvPoint2D64f* from = reinterpret_cast<const CvPoint2D64f*>(m1->data.ptr);
    const CvPoint2D64f* to   = reinterpret_cast<const CvPoint2D64f*>(m2->data.ptr);
    const double* F = model->data.db;
    float* err = error->data.fl;

    for(int i = 0; i < count; i++)
    {
      const CvPoint2D64f& f = from[i];
      const CvPoint2D64f& t = to[i];

      double a = F[0]*f.x + F[1]*f.y + F[2] - t.x;
      double b = F[3]*f.x + F[4]*f.y + F[5] - t.y;

      err[i] = (float)sqrt(a*a + b*b);
    }
  }

  bool Affine2DEstimator::runRANSAC(const CvMat* m1, const CvMat* m2, CvMat* model,
          CvMat* mask0, double reprojThreshold,
          double confidence, int maxIters)
  {
    bool result = false;
    cv::Ptr<CvMat> mask = cvCloneMat(mask0);
    cv::Ptr<CvMat> models, err, tmask;
    cv::Ptr<CvMat> ms1, ms2;

    int iter, niters = maxIters;
    int count = m1->rows*m1->cols, maxGoodCount = 0;
    CV_Assert(CV_ARE_SIZES_EQ(m1, m2) && CV_ARE_SIZES_EQ(m1, mask));

    if(count < modelPoints)
      return false;

    models = cvCreateMat(modelSize.height*maxBasicSolutions, modelSize.width, CV_64FC1);
    err = cvCreateMat(1, count, CV_32FC1);
    tmask = cvCreateMat(1, count, CV_8UC1);

    if(count > modelPoints)
    {
      ms1 = cvCreateMat(1, modelPoints, m1->type);
      ms2 = cvCreateMat(1, modelPoints, m2->type);
    }
    else
    {
      niters = 1;
      ms1 = cvCloneMat(m1);
      ms2 = cvCloneMat(m2);
    }

    for(iter = 0; iter < niters; iter++)
    {
      int i, goodCount, nmodels;
      if(count > modelPoints)
      {
        bool found = getSubset(m1, m2, ms1, ms2, 300);
        if(!found)
        {
                if(iter == 0)
                        return false;
                break;
        }
      }

      nmodels = runKernel(ms1, ms2, models);
      if(nmodels <= 0)
              continue;
      for(i = 0; i < nmodels; i++)
      {
        CvMat model_i;
        cvGetRows(models, &model_i, i*modelSize.height, (i+1)*modelSize.height);
        goodCount = findInliers(m1, m2, &model_i, err, tmask, reprojThreshold);

        if(goodCount > MAX(maxGoodCount, modelPoints-1))
        {
          std::swap(tmask, mask);
          cvCopy(&model_i, model);
          maxGoodCount = goodCount;
          niters = cvRANSACUpdateNumIters(confidence,
                  (double)(count - goodCount)/count, modelPoints, niters);
        }
      }
    }

    if(maxGoodCount > 0)
    {
      if(mask != mask0)
              cvCopy(mask, mask0);
      result = true;
    }

    return result;
  }

  cv::Mat getAffineTransform64f(const cv::Point2d src[], const cv::Point2d dst[])
  {
    cv::Mat M(2, 3, CV_64F), X(6, 1, CV_64F, M.data);
    double a[6*6], b[6];
    cv::Mat A(6, 6, CV_64F, a), B(6, 1, CV_64F, b);

    for(int i = 0; i < 3; i++)
    {
      int j = i*12;
      int k = i*12+6;
      a[j] = a[k+3] = src[i].x;
      a[j+1] = a[k+4] = src[i].y;
      a[j+2] = a[k+5] = 1;
      a[j+3] = a[j+4] = a[j+5] = 0;
      a[k] = a[k+1] = a[k+2] = 0;
      b[i*2] = dst[i].x;
      b[i*2+1] = dst[i].y;
    }

    solve(A, B, X);
    return M;
  }

  int Affine2DEstimator::runKernel( const CvMat* m1, const CvMat* m2, CvMat* model )
  {  
    const cv::Point2d* from = reinterpret_cast<const cv::Point2d*>(m1->data.ptr);
    const cv::Point2d* to   = reinterpret_cast<const cv::Point2d*>(m2->data.ptr);
    cv::Mat M0 = cv::cvarrToMat(model);
    cv::Mat M=getAffineTransform64f(from,to);
    CV_Assert( M.size() == M0.size() );
    M.convertTo(M0, M0.type());

    return model!=NULL?1:0;
  }

  int estimateAffine2D(cv::InputArray _from, cv::InputArray _to,
                      cv::OutputArray _out, cv::OutputArray _inliers,
                      double param1, double param2)
  {
    cv::Mat from = _from.getMat(), to = _to.getMat();
    int count = from.checkVector(2, CV_32F);

    CV_Assert(count >= 0 && to.checkVector(2, CV_32F) == count);

    _out.create(2, 3, CV_64F);
    cv::Mat out = _out.getMat();

    _inliers.create(count, 1, CV_8U, -1, true);
    cv::Mat inliers = _inliers.getMat();
    inliers = cv::Scalar::all(1);

    cv::Mat dFrom, dTo;
    from.convertTo(dFrom, CV_64F);
    to.convertTo(dTo, CV_64F);

    CvMat F2x3 = out;
    CvMat mask  = inliers;
    CvMat m1 = dFrom;
    CvMat m2 = dTo;

    const double epsilon = std::numeric_limits<double>::epsilon();
    param1 = param1 <= 0 ? 3 : param1;
    param2 = (param2 < epsilon) ? 0.99 : (param2 > 1 - epsilon) ? 0.99 : param2;

    return Affine2DEstimator().runRANSAC(&m1, &m2, &F2x3, &mask, param1, param2);
  }

  bool Affine2DEstimator::getSubset(const CvMat* m1, const CvMat* m2,
                                    CvMat* ms1, CvMat* ms2, int maxAttempts)
    {
    cv::AutoBuffer<int> _idx(modelPoints);
    int* idx = _idx;
    int i = 0, j, k, idx_i, iters = 0;
    int type = CV_MAT_TYPE(m1->type), elemSize = CV_ELEM_SIZE(type);
    const int *m1ptr = m1->data.i, *m2ptr = m2->data.i;
    int *ms1ptr = ms1->data.i, *ms2ptr = ms2->data.i;
    int count = m1->cols*m1->rows;

    assert(CV_IS_MAT_CONT(m1->type & m2->type) && (elemSize % sizeof(int) == 0));
    elemSize /= sizeof(int);

    for(; iters < maxAttempts; iters++)
    {
      for(i = 0; i < modelPoints && iters < maxAttempts;)
      {
        idx[i] = idx_i = cvRandInt(&rng) % count;
        for( j = 0; j < i; j++ )
          if( idx_i == idx[j] )
            break;
        if(j < i)
          continue;
        for(k = 0; k < elemSize; k++)
        {
          ms1ptr[i*elemSize + k] = m1ptr[idx_i*elemSize + k];
          ms2ptr[i*elemSize + k] = m2ptr[idx_i*elemSize + k];
        }
        if(checkPartialSubsets && (!checkSubset(ms1, i+1) || !checkSubset(ms2, i+1)))
        {
          iters++;
          continue;
        }
        i++;
      }
      if(!checkPartialSubsets && i == modelPoints &&
              (!checkSubset(ms1, i) || !checkSubset(ms2, i)))
              continue;
      break;
    }

    return i == modelPoints && iters < maxAttempts;
  }


  bool Affine2DEstimator::checkSubset(const CvMat* ms1, int count)
  {
    int j, k, i, i0, i1;
    CvPoint2D64f* ptr = (CvPoint2D64f*)ms1->data.ptr;

    assert(CV_MAT_TYPE(ms1->type) == CV_64FC2);

    if(checkPartialSubsets)
      i0 = i1 = count - 1;
    else
      i0 = 0, i1 = count - 1;

    for(i = i0; i <= i1; i++)
    {
      // check that the i-th selected point does not belong
      // to a line connecting some previously selected points
      for(j = 0; j < i; j++)
      {
        double dx1 = ptr[j].x - ptr[i].x;
        double dy1 = ptr[j].y - ptr[i].y;
        for(k = 0; k < j; k++)
        {
          double dx2 = ptr[k].x - ptr[i].x;
          double dy2 = ptr[k].y - ptr[i].y;
          if(fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
            break;
        }
        if(k < j)
          break;
      }
      if(j < i)
        break;
    }

    return i >= i1;
  }

  Affine2DEstimator::Affine2DEstimator() : modelPoints(3),modelSize(cvSize(3, 2)),maxBasicSolutions(1)
  {
    checkPartialSubsets = true;
    rng = cvRNG(-1);
  }

}
