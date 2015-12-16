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

    QObject::connect(
      &gui_connector_, SIGNAL(requestMap(QString, bool)),
      this, SLOT(receiveMapfromService(QString, bool)));

    QObject::connect(
      &gui_connector_,SIGNAL(loadDefaultMaps()),
      this, SLOT(receiveMapsFromServer()));

    QObject::connect(
        &validation_connector_, SIGNAL(changeXPos(double)),
        this, SLOT(moveMapHorizontally(double)));

    QObject::connect(
        &validation_connector_, SIGNAL(changeYPos(double)),
        this, SLOT(moveMapVertically(double)));
 
    QObject::connect(
        &validation_connector_, SIGNAL(changeRotation(int)),
        this, SLOT(rotateMap(int)));

    QObject::connect(
        &validation_connector_, SIGNAL(changeScale(double)),
        this, SLOT(scaleMap(double)));

    QObject::connect(
        &validation_connector_, SIGNAL(changeTransparency(double)),
        this, SLOT(changeMapTransparency(double)));

    QObject::connect(
        &validation_connector_, SIGNAL(omseMetricNeeded()),
        this, SLOT(requestOmseMetric()));

    QObject::connect(
        &validation_connector_, SIGNAL(cmseMetricNeeded()),
        this, SLOT(requestCmseMetric()));

    QObject::connect(
        &map_connector_, SIGNAL(mapPosChanged(qreal, qreal)),
        &validation_connector_, SLOT(showMapPosition(qreal, qreal)));

    QObject::connect(
        &map_connector_, SIGNAL(mapRotationChanged(qreal)),
        &validation_connector_, SLOT(showMapRotation(qreal)));

    QObject::connect(
        &map_connector_, SIGNAL(mapScaleChanged(qreal)),
        &validation_connector_, SLOT(showMapScale(qreal)));

  QObject::connect(
        &map_connector_, SIGNAL(mapTransparencyChanged(double)),
        &validation_connector_, SLOT(showMapTransparency(double)));

   timer_ = new QTimer(this);
    connect(
      timer_, SIGNAL(timeout()),
      this, SLOT(updateMapInternal()));

    elapsed_time_.start();

    map_initialized_ = true;
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
      gui_connector_.addToGrid(validation_connector_.getLoader(), 0, 0, 0, 0);
      
      initial_map_ = running_map_ = QImage((
        ogm_gui_tools::getRosPackagePath("ogm_resources") +
        std::string("/maps/slam_env_1.png")).c_str());

      initial_slam_map_ = slam_map_ = QImage((
        ogm_gui_tools::getRosPackagePath("ogm_resources") +
        std::string("/maps/slam_final_1.png")).c_str());

      nav_msgs::OccupancyGrid map;
      map.info.width = initial_map_.width();
      map.info.height = initial_map_.height();
      map.info.resolution = 0.02;
      maps_.groundTruthMap = map;

      map.info.width = initial_slam_map_.width();
      map.info.height = initial_slam_map_.height();
      map.info.resolution = 0.02;
      maps_.slamMap = map;

      validation_connector_.updateMapInfo(maps_);
      validation_connector_.resetMapProperties();

      map_connector_.updateImage(&running_map_, true);
      map_connector_.updateImage(&slam_map_, false);

      gui_connector_.addToGrid(map_connector_.getLoader(), 0, 1, 0, 0);

      gui_connector_.setGridColumnStretch(1, 5);
      gui_connector_.setGridColumnStretch(0, 2);

      //gui_connector_.addToGrid(alignment_connector_.getLoader(), 1, 0, 0, 0);

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
  @brief Receives the defaut maps as loaded from ogm_server.
  @return void
  **/
  void CGuiController::receiveMapsFromServer()
  {
    while(map_lock_)
    {
      usleep(100);
    }
    map_lock_ = true;
    
    ros::ServiceClient client;
    ogm_msgs::LoadMaps srv;

   while (!ros::service::waitForService
       ("/ogm_server/load_maps", ros::Duration(.1)) &&
         ros::ok())
    {
       ROS_WARN
         ("Trying to register to /ogm_server/load_maps...");
    }

    client = n_.serviceClient<ogm_msgs::LoadMaps>
       ("/ogm_server/load_maps", true);

    srv.request.request = "Send Maps";

    if (client.call(srv)) 
    {
      ROS_INFO("Maps successfully loaded from server");
       maps_.groundTruthMap = srv.response.groundTruthMap;
       maps_.slamMap = srv.response.slamMap;
    }
    else
    {
       ROS_ERROR("Could not load Maps, maybe already loaded...");
    }

    initial_map_ = running_map_ = QImage(maps_.groundTruthMap.info.width, maps_.groundTruthMap.info.height, QImage::Format_RGB32);

    initial_slam_map_ = slam_map_ = QImage(maps_.slamMap.info.width, maps_.slamMap.info.height, QImage::Format_RGB32);

    running_map_ = convertOccupancyGridToQImage(running_map_, maps_.groundTruthMap);
    slam_map_ = convertOccupancyGridToQImage(slam_map_, maps_.slamMap);

    initial_map_ = running_map_;
    initial_slam_map_ = slam_map_;

    validation_connector_.updateMapInfo(maps_);
    validation_connector_.resetMapProperties();

    map_connector_.resetMap();

    elapsed_time_.start();

    map_initialized_ = true;
    map_connector_.setMapInitialized(true);
    gui_connector_.setMapInitialized(true);

    timer_->start(50);

    map_lock_ = false;
  }

  /**
  @brief Receives the occupancy grid map from ogm_server.
  @param file_name [QString] The name of the mapFile to be loaded
  @param groundTruth [bool] var that shows if the map is groundTruth
  @return void
  **/
  void CGuiController::receiveMapfromService(QString file_name, bool groundTruth)
  {
    while(map_lock_)
    {
      usleep(100);
    }
    map_lock_= true;

    ros::ServiceClient client;
    ogm_msgs::LoadExternalMap srv;

    while (!ros::service::waitForService
       ("/ogm_server/load_static_map_external", ros::Duration(.1)) &&
         ros::ok())
    {
       ROS_WARN
         ("Trying to register to /ogm_server/load_static_map_external...");
    }

    client = n_.serviceClient<ogm_msgs::LoadExternalMap>
       ("/ogm_server/load_static_map_external", true);

    srv.request.mapFile = file_name.toStdString();
    srv.request.groundTruth = groundTruth;

    if (client.call(srv)) 
    {
      if(groundTruth)
      {
        ROS_INFO("Ground Truth Map successfully loaded from GUI");
        maps_.groundTruthMap = srv.response.map;
      }
      else
      {
        ROS_INFO("Slam - produced Map successfully loaded from GUI");
        maps_.slamMap = srv.response.map;
      }
    }
    else
    {
       ROS_ERROR("Could not load Map, maybe already loaded...");
    }
  
    if(groundTruth)
    {
       initial_map_ = running_map_ = QImage(maps_.groundTruthMap.info.width, maps_.groundTruthMap.info.height, QImage::Format_RGB32);
       running_map_ = convertOccupancyGridToQImage(running_map_, maps_.groundTruthMap);
       initial_map_ = running_map_;
      }

    else
    {
       initial_slam_map_ = running_map_ = QImage(maps_.slamMap.info.width, maps_.slamMap.info.height, QImage::Format_RGB32);
       running_map_ = convertOccupancyGridToQImage(running_map_, maps_.slamMap);
      initial_slam_map_ = running_map_;
    }

    validation_connector_.updateMapInfo(maps_);

    validation_connector_.resetMapProperties();
    map_connector_.resetMap();

    elapsed_time_.start();

    map_initialized_ = true;
    map_connector_.setMapInitialized(true);
    gui_connector_.setMapInitialized(true);

    timer_->start(50);

    map_lock_=false;
  }

  /**
  @brief Converts nav_msgs/OccupancyGrid to QImage
  @param running_map [QImage] The QImage to be drawn the map
  @param map_msg [nav_msgs::OccupancyGrid] the OccupancyGrid
  @return QImage
  **/
  QImage CGuiController::convertOccupancyGridToQImage(QImage running_map, nav_msgs::OccupancyGrid map_msg)
  {
    QPainter painter(&running_map);
    int d(0);
    QColor c;
    for( unsigned int i = 0 ; i < map_msg.info.width ; i++ )
    {
      for( unsigned int j = 0 ; j < map_msg.info.height ; j++ )
      {
        if( map_msg.data[j * map_msg.info.width + i] == -1 )
        {
          c = QColor(127,127,127);
        }
        else
        {
          d = (100.0 - map_msg.data[j * map_msg.info.width + i]) / 100.0 * 255.0;
          c = QColor(d, d, d);
        }
        painter.setPen(c);
        painter.drawPoint(i, j);
      }
    }
    int originx = map_msg.info.origin.position.x / map_msg.info.resolution;
    int originy = map_msg.info.origin.position.y / map_msg.info.resolution;
    painter.setPen(Qt::blue);
    painter.drawLine(originx, originy - 20, originx, originy + 20);
    painter.drawLine(originx - 20, originy, originx + 20, originy);

    return running_map;
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
      slam_map_ = initial_slam_map_;
    if(gui_connector_.isGridEnabled())
    {
      map_connector_.drawGrid(&running_map_, maps_.groundTruthMap.info.resolution);
      map_connector_.drawGrid(&slam_map_, maps_.slamMap.info.resolution);
    }
    map_connector_.updateImage(&running_map_, true);
    map_connector_.updateImage(&slam_map_, false);

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

  void CGuiController::moveMapHorizontally(double x)
  {
    map_connector_.setMapXposition(x);
  }

  void CGuiController::moveMapVertically(double y)
  {
    map_connector_.setMapYposition(y);
  }

  void CGuiController::rotateMap(int r)
  {
    map_connector_.setMapRotation(r);
  }

  void CGuiController::scaleMap(double s)
  {
    map_connector_.setMapScale(s);
  }

  void CGuiController::changeMapTransparency(double t)
  {
    map_connector_.setMapTransparency(t);
  }

  void CGuiController::requestOmseMetric()
  {
    ros::ServiceClient client;
    ogm_msgs::GuiRequestEvaluation srv;

    while (!ros::service::waitForService
       ("/ogm_server/map_evaluation", ros::Duration(.1)) &&
         ros::ok())
    {
       ROS_WARN
         ("Trying to register to /ogm_server/map_evaluation...");
    }

    client = n_.serviceClient<ogm_msgs::GuiRequestEvaluation>
       ("/ogm_server/map_evaluation", true);

    QPointF pos =  map_connector_.getPosition();
    qreal rot =  map_connector_.getRotation(); 
    qreal scale =  map_connector_.getScale();
    srv.request.method = "OMSE";
    srv.request.transform.pose.x = pos.x();
    srv.request.transform.pose.y = pos.y();
    srv.request.transform.pose.theta =rot;
    srv.request.transform.scale = scale;

    if (client.call(srv)) 
    {
      omse_ = srv.response.result;
      ROS_INFO_STREAM("[ogm_gui] OMSE metric successfully calculated OMSE=" << omse_);
      validation_connector_.displayOmseResult(omse_);
    }
    else
    {
       ROS_ERROR("[ogm_gui] OMSE not received..");
    }
  }

  void CGuiController::requestCmseMetric()
  {
    ros::ServiceClient client;
    ogm_msgs::GuiRequestEvaluation srv;

    while (!ros::service::waitForService
       ("/ogm_server/map_evaluation", ros::Duration(.1)) &&
         ros::ok())
    {
       ROS_WARN
         ("Trying to register to /ogm_server/map_evaluation...");
    }

    client = n_.serviceClient<ogm_msgs::GuiRequestEvaluation>
       ("/ogm_server/map_evaluation", true);

    QPointF pos =  map_connector_.getPosition();
    qreal rot =  map_connector_.getRotation(); 
    qreal scale =  map_connector_.getScale();
    srv.request.method = "CMSE";
    srv.request.transform.pose.x = pos.x();
    srv.request.transform.pose.y = pos.y();
    srv.request.transform.pose.theta =rot;
    srv.request.transform.scale = scale;

    if (client.call(srv)) 
    {
      cmse_ = srv.response.result;
      ROS_INFO_STREAM("[omg_gui] Corner metric successfully calculated CMSE=" << omse_);
      validation_connector_.displayCmseResult(cmse_);
    }
    else
    {
       ROS_ERROR("[ogm_gui] CMSE not received..");
    }
  }

}


