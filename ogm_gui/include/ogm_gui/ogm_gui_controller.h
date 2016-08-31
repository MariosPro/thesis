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

#ifndef OGM_GUI_CONTROLLER
#define OGM_GUI_CONTROLLER

#include <iostream>
#include <cstdlib>
#include <boost/thread.hpp>

#include "nav_msgs/OccupancyGrid.h"

#include <QtCore/QThread>
#include <QtCore/QTime>
#include <QtCore/QTimer>

//#include <ogm_communications/MapsMsg.h>
#include <ogm_communications/MapPublish.h>
#include <ogm_communications/LoadExternalMap.h>
#include <ogm_communications/LoadMapsFromServer.h>
#include <ogm_communications/GuiRequestEvaluation.h>
#include <ogm_communications/GuiRequestStructuralEvaluation.h>
#include "ogm_gui/ogm_gui_connector.h"
#include "ogm_gui/ogm_validation_connector.h"
#include "ogm_gui/ogm_map_connector.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{

  /**
  @class GuiController
  @brief The main controller for the OGM GUI. Inherits QThread
  **/
  class CGuiController :
    public QThread
  {
    Q_OBJECT

    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int  argc_;
      //!< Input arguments
      char** argv_;

      //!< Prevents concurrent map writing
      bool map_lock_;
      //!< Prevents actions before map initializes
      bool map_initialized_;

     //!< ROS subscriber for occupancy grid map
      ros::Subscriber map_subscriber_;

      //!< The ROS node handle
      ros::NodeHandle n_;

     //!< The MapsMsg hold the two Maps
     ogm_communications::MapsMsg maps_;

      //!< Timer for the drawing event
      QTimer* timer_;

      //!< Elapsed time from the experiment's start
      QTime elapsed_time_;

     //!< QImage created one time, containing the OGM
      QImage initial_map_;

      //!< QImage that initiates as initial_map and the elements are painted on it
      QImage running_map_;

    //!< QImage created one time, contains the map produced by a SLAM algorithm
     QImage initial_slam_map_;

      //!< QImage that contains the map produced by a SLAM algorithm and the
      //!< elements are painted on it
      QImage slam_map_;

      //!< Object of CGuiConnector
      CGuiConnector gui_connector_;

      //!< Object of CValidationConnector
      CValidationConnector validation_connector_;

      //!< Object of CAlignmentConnector
      //CAlignmentConnector alignment_connector_;

      //!< Object of CMapConnector
      CMapConnector map_connector_;

      //!< Metric result
      double metricResult_;

      //!< structural evaluation result
      double structuralEvaluationResult_;

      //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CGuiController(int argc,char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiController(void);

      /**
      @brief Sets up the main window widgets
      @return void
      **/
      void setupWidgets(void);

      /**
      @brief Initializes the Qt event connections and ROS subscribers and publishers
      @return void
      **/
      void initializeCommunications(void);

      /**
      @brief Receives the occupancy grid map from ogm_server. Connects to "map" ROS topic
      @param msg [const ogm_communications::MapPublish&] The OGM message
      @return void
      **/
      void receiveMap(const ogm_communications::MapPublish& msg);

      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();

      /**
      @brief Converts nav_msgs/OccupancyGrid to QImage
      @param running_map [QImage] The QImage to be drawn the map
      @param map_msg [nav_msgs::OccupancyGrid] the OccupancyGrid
      @return QImage
      **/
      Q_INVOKABLE QImage convertOccupancyGridToQImage(QImage running_map, nav_msgs::OccupancyGrid map_msg);

    //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief Receives the occupancy grid map from ogm_server.
      @param file_name [QString] The name of the mapFile to be loaded
      @param groundTruth [bool] var that shows if the map is groundTruth
      @return void
      **/
      void requestMap(QString file_name, bool groundTruth);

      /**
      @brief Receives the defaut maps as loaded from ogm_server.
      @return void
      **/
      void receiveMapsFromServer();

      /**
      @brief Updates the map to be shown in GUI. Connects to the timeout signal of timer_
      @return void
      **/
      void updateMapInternal(void);

      void moveMapHorizontally(double x);

      void moveMapVertically(double y);

      void rotateMap(int r);

      void scaleMap(double s);

      void changeMapTransparency(double t);

      void requestMetric(QString metricMethod);

      void requestStructuralEvaluation();

      //------------------------------------------------------------------------//
    Q_SIGNALS:

  };
}

#endif

