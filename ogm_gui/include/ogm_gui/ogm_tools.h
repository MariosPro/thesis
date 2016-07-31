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

#ifndef OGM_TOOLS
#define OGM_TOOLS
#define _USE_MATH_DEFINES

#include <cmath>
#include "ros/ros.h"
#include <ros/package.h>
// #include <tf/transform_listener.h>

#include <QtUiTools/QUiLoader>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QTime>

#include <QtWidgets/QMenu>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtGui/QImage>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtGui/QPainter>
#include <QtGui/QPixmap>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGraphicsPixmapItem>
//#include <QtGui/QWindowsStyle>
#include <QtWidgets/QInputDialog>
#include <QtGui/QMouseEvent>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QTimeEdit>
#include <QtWidgets/QInputDialog>
#include <QtGui/QFont> 

#include <geometry_msgs/Pose2D.h>
/* #include <geometry_msgs/Point.h> */
/* #include <geometry_msgs/Twist.h> */

// #include <ogm_msgs/LoadExternalMap.h>


/**
@namespace ogm_gui_tools
@brief The namespace for OGM GUI tools
**/
namespace ogm_gui_tools
{
  /**
  @brief Returns the global path of the ROS package provided
  @param package [std::string] The ROS package
  @return std::string : The global path of the specific package
  **/
  std::string getRosPackagePath(std::string package);

  /**
  @brief Transforms the milliseconds in literal representation
  @param ms [int] The time in ms
  @return QString : The literal representation of time given
  **/
  QString getLiteralTime(int ms);

  /**
  @brief Converts an angle from rads to degrees
  @param angle [float] An angle in rads
  @return float : The angle in degrees
  **/
  float angleRadToDegrees(float angle);

  /**
  @brief Converts an angle from degrees to rads
  @param angle [float] An angle in degrees
  @return float : The angle in rads
  **/
  float angleDegreesToRad(float angle);
  /**
  @brief Prints a ROS pose2d msg
  @param msg [geometry_msgs::Pose2D &] The message
  @return void
  **/
  void printPose2D(geometry_msgs::Pose2D &msg);
}

#endif
