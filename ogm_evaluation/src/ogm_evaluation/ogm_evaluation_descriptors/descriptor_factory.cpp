/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2016 OGM Validator
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

#include <string>
#include "ogm_evaluation/ogm_evaluation_descriptors/descriptor_factory.h"
#include "ogm_evaluation/ogm_evaluation_descriptors/radius_statistics_descriptor.h"
#include "ogm_evaluation/ogm_evaluation_descriptors/circle_intersection_descriptor.h"

namespace ogm_evaluation
{
   /**
   * @brief The main factory method that creates the different descriptors
   * @param name [std::string] The name of the descriptor
   */
  DescriptorExtractor* DescriptorFactory::create(std::string name)
  {
    if (name == "RADIUS STATISTICS") return new RadiusStatisticsDescriptor();
    else if (name == "CIRCLE INTERSECTIONS") return new CircleIntersectionDescriptor();

    else
      {
    /*    ROS_FATAL_STREAM("[OGM_EVALUATION]: Invalid metric method!"*/
            /*<< " Detection cannot continue!");*/
        return NULL;
      }
      return NULL;
  }
}  // namespace ogm_evaluation
