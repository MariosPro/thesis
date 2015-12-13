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
#include "ogm_evaluation/ogm_evaluation.h"

/**
@brief Main function of the OGM evaluation node
@param argc [int] Number of input arguments
@param argv [char**] Input arguments
@return int
**/
int main(int argc, char** argv) {

  ros::init(argc, argv, "ogm_evaluation");

  ogm_evaluation::MapEvaluator evaluator(argc, argv);

  ros::spin();

  return 0;
}
