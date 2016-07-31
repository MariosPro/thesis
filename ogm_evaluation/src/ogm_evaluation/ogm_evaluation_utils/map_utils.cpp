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

#include "ogm_evaluation/ogm_evaluation_utils/map_utils.h"

namespace ogm_evaluation
{
  /**
  @brief Default Constructor
  @return void
  **/
  MapUtils::MapUtils()
  {

  }
  
  /**
  @brief Calculates the minimum distance of all free and unknown cells from the closest occupied cells
  @param map [const cv::Mat&] the map cv::Mat
  @param _brushfire [** int] the array the distances to be saved
  @return void
  **/
  void MapUtils::brushfireSearch(const cv::Mat& map, int** brushfire)
  {
    for(int i = 0 ; i < map.rows ; i++)
    {
      for(int j = 0 ; j < map.cols; j++)
      {
        if(map.at<uchar>(i, j) >= 127) // the wave can be spread in free and unknown space
          brushfire[i][j] = -1;
        else
          brushfire[i][j] = 0;
      }
    }

    // Create brushfire (Manhattan distance transformation)
    bool foundWave = true;
    int currentWave = 0; //initial wave is Obstacles (0)

    while(foundWave)
    {
      foundWave = false;
      for (int i = 0; i < map.rows; i++)
        for(int j = 0; j < map.cols; j++)
          if(brushfire[i][j] == currentWave)
          {
            foundWave = true;
            if(i > 0 ) //This code checks the array bounds heading WEST
              if(brushfire[i - 1][j] == -1)//This code checks the WEST direction
                brushfire[i - 1][j] = currentWave + 1;

            if(i < (map.rows -1)) //This code checks the array bounds heading EAST
              if(brushfire[i + 1][j] == -1)//This code checks the EAST direction
                brushfire[i + 1][j] = currentWave + 1;

            if(j > 0) //This code checks the array bounds heading SOUTH
              if(brushfire[i][j - 1] == -1)//This code checks the SOUTH direction
                brushfire[i][j - 1] = currentWave + 1;
            if(j < (map.cols - 1)) //This code checks the array bounds heading NORTH
              if(brushfire[i][j + 1] == -1)//This code checks the NORTH direction
                brushfire[i][j + 1] = currentWave + 1;
          }
      currentWave++;
    }
  }

  /**
  @brief Calculates the average of the free cells on the brushfire array 
  @param _brushfire [** int] the array of the distances
  @return double the avg distance
  **/
  double MapUtils::meanBrushfireDistance(const cv::Mat& map, int** brushfire)
  {
    double sum = 0;
    int frees = 0;
    for (int i = 0; i < map.rows; i++)
      for (int j = 0;  j < map.cols; j++)
      {
        if(map.at<uchar>(i, j) != 127)
        {
          sum += brushfire[i][j];
          frees++;
        }
      }
    return sum/frees;
  }
}  // namespace ogm_evaluation
