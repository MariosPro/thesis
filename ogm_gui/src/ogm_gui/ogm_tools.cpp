/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
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

#include "ogm_gui/ogm_tools.h"

namespace ogm_gui_tools
{
  /**
  @brief Returns the global path of the ROS package provided
  @param package [std::string] The ROS package
  @return std::string : The global path of the specific package
  **/
  std::string getRosPackagePath(std::string package)
  {
    return ros::package::getPath(package.c_str());
  }

  /**
  @brief Converts an angle from rads to degrees
  @param angle [float] An angle in rads
  @return float : The angle in degrees
  **/
  float angleRadToDegrees(float angle)
  {
    return angle * 180.0 / M_PI;
  }

  /**
  @brief Converts an angle from degrees to rads
  @param angle [float] An angle in degrees
  @return float : The angle in rads
  **/
  float angleDegreesToRad(float angle)
  {
    return angle / 180.0 * M_PI;
  }

  /**
  @brief Transforms the milliseconds in literal representation
  @param ms [int] The time in ms
  @return QString : The literal representation of time given
  **/
  QString getLiteralTime(int ms)
  {
    QString str;
    int h = ms / (1000 * 60 * 60);
    int m = ms / (1000 * 60) - h * 60;
    int s = ms / 1000 - h * 60 * 60 - m * 60;
    int ms_ = ms - h * 60 * 60 * 1000 - m * 1000 * 60 - s * 1000;
    if(h)
    {
      str += QString().setNum(h) + QString(" h ");
    }
    if(m || h)
    {
      str += QString().setNum(m) + QString(" min ");
    }
    if(s || h || m)
    {
      str += QString().setNum(s) + QString(" sec ");
    }
    if(ms_ || s || h || m)
    {
      str += QString().setNum(ms_) + QString(" ms");
    }
    return str;
  }

  /**
  @brief Prints a ROS pose2d msg
  @param msg [geometry_msgs::Pose2D &] The message
  @return void
  **/
  void printPose2D(geometry_msgs::Pose2D &msg)
  {
    ROS_ERROR("Pose 2D :");
    ROS_ERROR("\tx : %f",msg.x);
    ROS_ERROR("\ty : %f",msg.y);
    ROS_ERROR("\ttheta : %f",msg.theta);
  }
 }
