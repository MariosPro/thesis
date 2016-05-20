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
   * Manos Tsardoulias, etsardou@gmail.com
******************************************************************************/

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>

/**
@namespace ogm_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace ogm_evaluation
{
  /**
  @struct Parameters
  @brief Provides flexibility by parameterizing variables needed by the
  ogm_evaluation package
  **/
  struct Parameters
  {
    //!< the Feature Detector to be used
    std::string detector;

    //!< the Descriptor Extractor to be used
    std::string descriptor;

    //!< the Descriptor Matcher to be used
    std::string matcher;

    //!< the matching method to be used by the matcher
    std::string matchingMethod;

    //!< the closestPoint method to be used (in OMSE Metric)
    std::string closestPointMethod;

    //!< the dist norm to be used to calculate distances
    std::string distNorm;

    //!< the matchingRatio to be used in matching descriptors
    double matchingRatio;

    //!< the maximum Ransac Reprojection Error for rejecting outliers
    double ransacReprjError;

    //!< indicates if scaling (using brushfire mean distance ratio) is to performed
    bool scaleMapsBrushfire;

    //!< indicates if map is going to be binary
    bool binary;

    //!< indicates if manual alignment has been performed
    bool manualAlignment;

    //!< indicates if on benchmarking mode
    bool benchmarking;

  };
}
#endif

