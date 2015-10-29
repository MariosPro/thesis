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

#include "ogm_gui/ogm_gui_connector.h"
#include "ogm_gui/ogm_validation_connector.h"
#include "ogm_gui/ogm_alignment_connector.h"
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

      //!< ROS tf transform listener
      //tf::TransformListener listener_;

      //!< The occypancy grid map
      nav_msgs::OccupancyGrid map_msg_;

      //!< Timer for the drawing event
      QTimer* timer_;

      //!< Elapsed time from the experiment's start
      QTime elapsed_time_;

     //!< QImage created one time, containing the OGM
      QImage initial_map_;

      //!< QImage that initiates as initial_map and the elements are painted on it
      QImage running_map_;

      //!< Object of CGuiConnector
      CGuiConnector gui_connector_;

      //!< Object of CValidationConnector
      CValidationConnector validation_connector_;

      //!< Object of CAlignmentConnector
      CAlignmentConnector alignment_connector_;

      //!< Object of CMapConnector
      CMapConnector map_connector_;

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
      @param msg [const nav_msgs::OccupancyGrid&] The OGM message
      @return void
      **/
      void receiveMap(const nav_msgs::OccupancyGrid& msg);

      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();
    //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief Performs zoom in when the button is pressed. Connects to the CMapConnector::zoomInPressed signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void zoomInPressed(QPoint p);

      /**
      @brief Performs zoom out when the button is pressed. Connects to the CMapConnector::zoomOutPressed signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void zoomOutPressed(QPoint p);

      /**
      @brief Updates the map to be shown in GUI. Connects to the timeout signal of timer_
      @return void
      **/
      void updateMapInternal(void);

      /**
      @brief Informs CGuiController that click has performed in the map. Connects to the CMapConnector::itemClicked signal
      @param p [QPoint] The event point in map
      @param b [Qt::MouseButton] The mouse button used to trigger the event
      @return void
      **/
      void itemClicked(QPoint p,Qt::MouseButton b);

     /**
     @brief Qt slot that is called when the moveUpMapPressed signal is received
     @return void
     **/
     //void moveUpMap();

     /**
     @brief Qt slot that is called when the moveDownMapPressed signal is received
     @return void
     **/
     //void moveDownMap();

     /**
     @brief Qt slot that is called when the moveLeftMapPressed signal is received
     @return void
     **/
     //void moveLeftMap();

     /**
     @brief Qt slot that is called when the moveRightMapPressed signal is received
     @return void
     **/
     //void moveRightMap();

     /**
     @brief Qt slot that is called when the rotateLeftMapPressed signal is received
     @return void
     **/
     //void rotateLeftMap();

     /**
     @brief Qt slot that is called when the rotateRightMapPressed signal is received
     @return void
     **/
     //void rotateRightMap();

     /**
     @brief Qt slot that is called when the scaleMapPressed signal is received
     @return void
     **/
     //void scaleMap();

      //------------------------------------------------------------------------//
    Q_SIGNALS:

     /**
     @brief Is emmited when the map is going to be moveUp. Connects to the CMapConnector::moveUp slot
     @return void
     **/
     void moveUpMap(void);

     /**
     @brief Is emmited when the map is going to be moveDown. Connects to the CMapConnector::moveDown slot
     @return void
     **/
     void moveDownMap(void);

     /**
     @brief Is emmited when the map is going to be moveLeft. Connects to the CMapConnector::moveLeft slot
     @return void
     **/
     void moveLeftMap(void);

     /**
     @brief Is emmited when the map is going to be moveRight. Connects to the CMapConnector::moveRight slot
     @return void
     **/
     void moveRightMap(void);

     /**
     @brief Is emmited when the map is going to be rotateLeft. Connects to the CMapConnector::rotateLeft slot
     @return void
     **/
     void rotateLeftMap(void);

     /**
     @brief Is emmited when the map is going to be rotateRight. Connects to the CMapConnector::rotateRight slot
     @return void
     **/
     void rotateRightMap(void);

     /**
     @brief Is emmited when the map is going to be scaled. Connects to the CMapConnector::scale slot
     @return void
     **/
     void scaleMap(void);

  };
}

#endif

