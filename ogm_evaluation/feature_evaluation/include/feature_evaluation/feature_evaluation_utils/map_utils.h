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
#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include <opencv2/opencv.hpp>

/**
@namespace feature_evaluation
@brief The main namespace for OGM Evaluation
**/ 
namespace feature_evaluation
{
  /**
  @class MapUtils
  @brief A class that provides functions commonly used in maps
  **/ 
  class MapUtils
  {

    public:
      /**
      @brief Default Constructor
      @return void
      **/
      MapUtils();
    
      /**
      @brief Default destructor
      @return void
      **/
      virtual ~MapUtils(void) 
      {
      }

      /**
      @brief Calculates the minimum distance of all free and unknown cells from the closest       occupied cells
      @param map [const cv::Mat&] the map cv::Mat
      @param _brushfire [** int] the array the distances to be saved
      @return void
      **/
      void brushfireSearch(const cv::Mat& map, int** brushfire);

      /**
      @brief Calculates the average of the free cells on the brushfire array 
      @param _brushfire [** int] the array of the distances
      @return double the avg distance
      **/
      double meanBrushfireDistance(const cv::Mat& map, int** brushfire);


    private:
  };
}
#endif



