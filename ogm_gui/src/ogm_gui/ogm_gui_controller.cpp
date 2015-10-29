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

#include "ogm_gui/ogm_gui_controller.h"


namespace ogm_gui
{
  /**
  @brief Thread that performs the ros::spin functionality
  @return void
  **/
  void spinThreadFunction(void)
  {
    ros::spin();
  }

  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CGuiController::CGuiController(int argc,char **argv):
    gui_connector_(argc, argv),
    validation_connector_(argc, argv),
    alignment_connector_(argc, argv),
    map_connector_(argc, argv),
    argc_(argc),
    argv_(argv)
  {
    setupWidgets();

    map_lock_ = false;
    map_initialized_ = false;

  }

  /**
  @brief Default destructor
  @return void
  **/
  CGuiController::~CGuiController(void)
  {

  }

  /**
  @brief Initializes the Qt event connections and ROS subscribers and publishers
  @return void
  **/
  void CGuiController::initializeCommunications(void)
  {
    map_subscriber_ = n_.subscribe(
      "map",
      1,
      &CGuiController::receiveMap,
      this);

    QObject::connect(
      &gui_connector_,SIGNAL(setZoomInCursor(bool)),
      &map_connector_, SLOT(setCursorZoomIn(bool)));

    QObject::connect(
      &gui_connector_,SIGNAL(setZoomOutCursor(bool)),
      &map_connector_, SLOT(setCursorZoomOut(bool)));

    QObject::connect(
      &gui_connector_,SIGNAL(setAdjustedCursor(bool)),
      &map_connector_, SLOT(setCursorAdjusted(bool)));

    QObject::connect(
      &map_connector_,SIGNAL(zoomInPressed(QPoint)),
      this, SLOT(zoomInPressed(QPoint)));

    QObject::connect(
      &map_connector_,SIGNAL(zoomOutPressed(QPoint)),
      this, SLOT(zoomOutPressed(QPoint)));

    QObject::connect(
     &map_connector_,SIGNAL(itemClicked(QPoint,Qt::MouseButton)),
      this, SLOT(itemClicked(QPoint,Qt::MouseButton)));

    QObject::connect(
      &alignment_connector_, SIGNAL(moveUpMapPressed()),
      &map_connector_, SLOT(moveUp()));

    QObject::connect(
      &alignment_connector_, SIGNAL(moveDownMapPressed()),
      &map_connector_, SLOT(moveDown()));

    QObject::connect(
      &alignment_connector_, SIGNAL(moveLeftMapPressed()),
      &map_connector_, SLOT(moveLeft()));

    QObject::connect(
      &alignment_connector_, SIGNAL(moveRightMapPressed()),
       &map_connector_, SLOT(moveRight()));

    QObject::connect(
      &alignment_connector_, SIGNAL(rotateLeftMapPressed()),
      &map_connector_, SLOT(rotateLeft()));

    QObject::connect(
      &alignment_connector_, SIGNAL(rotateRightMapPressed()),
      &map_connector_, SLOT(rotateRight()));

    QObject::connect(
      &alignment_connector_, SIGNAL(scaleMapPressed()),
      &map_connector_, SLOT(scale()));

       timer_ = new QTimer(this);
    connect(
      timer_, SIGNAL(timeout()),
      this, SLOT(updateMapInternal()));

    elapsed_time_.start();

    map_initialized_ = true;
    map_connector_.setInitialImageSize(
      QSize(initial_map_.width(),initial_map_.height()));
    map_connector_.setMapInitialized(true);
    gui_connector_.setMapInitialized(true);

    timer_->start(50);

 }

  /**
  @brief Sets up the main window widgets
  @return void
  **/
  void CGuiController::setupWidgets(void)
  {
    {
      gui_connector_.addToGrid(validation_connector_.getLoader(), 0, 0, 0, 0);
    }
    {
      initial_map_ = running_map_ = QImage((
        ogm_gui_tools::getRosPackagePath("ogm_gui") +
        std::string("/resources/maps/sparse_obstacles.png")).c_str());

      map_msg_.info.width = initial_map_.width();
      map_msg_.info.height = initial_map_.height();
      //map_msg_.info.resolution = 0.02;

      map_connector_.updateImage(&running_map_);

      gui_connector_.addToGrid(map_connector_.getLoader(), 0, 1, -1, -1);

      gui_connector_.setGridColumnStretch(1, 100);
      gui_connector_.setGridColumnStretch(0, 0);

      gui_connector_.addToGrid(alignment_connector_.getLoader(), 1, 0, 0, 0);
    }
  }

  /**
  @brief Initializes the ROS spin and Qt threads
  @return bool
  **/
  bool CGuiController::init(void)
  {
    if ( ! ros::master::check() )
    {
      return false;
    }
    gui_connector_.show();

    initializeCommunications();
    boost::thread spinThread(&spinThreadFunction);
    return true;
  }

  /**
  @brief Receives the occupancy grid map from ogm_server. Connects to "map" \
  ROS topic
  @param msg [const nav_msgs::OccupancyGrid&] The OGM message
  @return void
  **/
  void CGuiController::receiveMap(const nav_msgs::OccupancyGrid& msg)
  {
    //ROS_INFO("RECEIVE MAP");
    map_msg_ = msg;
    initial_map_ = running_map_ =
      QImage(msg.info.width,msg.info.height,QImage::Format_RGB32);
    QPainter painter(&running_map_);
    int d(0);
    QColor c;
    for( unsigned int i = 0 ; i < msg.info.width ; i++ )
    {
      for( unsigned int j = 0 ; j < msg.info.height ; j++ )
      {
        if( msg.data[j * msg.info.width + i] == -1 )
        {
          c=QColor(127,127,127);
        }
        else
        {
          d = (100.0 - msg.data[j * msg.info.width + i]) / 100.0 * 255.0;
          c=QColor(d,d,d);
        }
        painter.setPen(c);
        painter.drawPoint(i,j);
      }
    }
    int originx = msg.info.origin.position.x / msg.info.resolution;
    int originy = msg.info.origin.position.y / msg.info.resolution;
    painter.setPen(Qt::blue);
    painter.drawLine(originx, originy - 20, originx, originy + 20);
    painter.drawLine(originx - 20, originy, originx + 20, originy);

    initial_map_ = running_map_;

    /*info_connector_.updateMapInfo( msg.info.width * msg.info.resolution,*/
                  //msg.info.height * msg.info.resolution,
                  /*msg.info.resolution);*/
    map_connector_.setInitialImageSize(
      QSize(initial_map_.width(),initial_map_.height()));

    elapsed_time_.start();

    map_initialized_ = true;
    map_connector_.setMapInitialized(true);
    gui_connector_.setMapInitialized(true);

    timer_->start(50);
  }

  /**
  @brief Performs zoom in when the button is pressed. Connects to the CMapConnector::zoomInPressed signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::zoomInPressed(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    map_connector_.updateZoom(p,true);
  }

  /**
  @brief Performs zoom out when the button is pressed. Connects to the CMapConnector::zoomOutPressed signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::zoomOutPressed(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    map_connector_.updateZoom(p,false);
  }

  /**
  @brief Updates the map to be shown in GUI. Connects to the timeout signal of timer_
  @return void
  **/
  void CGuiController::updateMapInternal(void)
  {
    while(map_lock_)
    {
      usleep(100);
    }
    map_lock_ = true;
    running_map_ = initial_map_;
    if(gui_connector_.isGridEnabled())
      map_connector_.drawGrid(&running_map_, 0.02); //map_msg_.info.resolution);

    map_connector_.updateImage(&running_map_);

      gui_connector_.setStatusBarMessage(
      QString("Time elapsed : ") +
      ogm_gui_tools::getLiteralTime(elapsed_time_.elapsed()));
    map_lock_ = false;

    //!< -----------------------------------------Check for close event
    if(gui_connector_.closeTriggered())
    {
      QEvent *e = gui_connector_.getCloseEvent();

      this->exit();
      gui_connector_.shutdown();
    }
  }

  /**
  @brief Informs CGuiController that click has performed in the map. Connects to the CMapConnector::itemClicked signal
  @param p [QPoint] The event point in map
  @param b [Qt::MouseButton] The mouse button used to trigger the event
  @return void
  **/
  void CGuiController::itemClicked(QPoint p,Qt::MouseButton b)
  {
    gui_connector_.uncheckZoomButtons();
    QPoint pointClicked = map_connector_.getGlobalPoint(p);
  }

  /**
  @brief Qt slot that is called when the moveUpMapPressed signal is received
  @return void
  **/
 /* void CGuiController::moveUpMap()*/
  //{
   //Q_EMIT moveUpMap();
  /*}*/

 /* [>**/
  //@brief Qt slot that is called when the moveDownMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::moveDownMap()
  //{
   //Q_EMIT moveDownMap();
  //}

  //[>*
  //@brief Qt slot that is called when the moveLeftMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::moveLeftMap()
  //{
   //Q_EMIT moveLeftMap();

  //[>*
  //@brief Qt slot that is called when the moveRightMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::moveRightMap()
  //{
   //Q_EMIT moveRightMap();
  //}

  //[>*
  //@brief Qt slot that is called when the rotateLeftMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::rotateLeftMap()
  //{
   //Q_EMIT rotateLeftMap();
  //}

  //[>*
  //@brief Qt slot that is called when the rotateRightMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::rotateRightMap()
  //{
   //Q_EMIT rotateRightMap();
  //}

  //[>*
  //@brief Qt slot that is called when the scaleMapPressed signal is received
  //@return void
  //**/
  //void CGuiController::scaleMap()
  //{
   //Q_EMIT scaleMap();
  /*}*/

}


