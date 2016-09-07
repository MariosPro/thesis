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

#include "ogm_gui/ogm_map_connector.h"

namespace ogm_gui{

  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CMapConnector::CMapConnector(int argc, char **argv):
    QObject(),
    loader_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    //map_state_ = NORMAL;

    //loader_.mapGraphicsView->setScaledContents(true);

    QObject::connect(
      loader_.ground_truth_map, SIGNAL(posChanged(qreal, qreal)),
      this, SIGNAL(mapPosChanged(qreal, qreal)));
 
    QObject::connect(
      loader_.ground_truth_map, SIGNAL(rotationChanged(qreal)),
      this, SIGNAL(mapRotationChanged(qreal)));
 
    QObject::connect(
      loader_.ground_truth_map, SIGNAL(scaleChanged(qreal)),
      this, SIGNAL(mapScaleChanged(qreal)));

    QObject::connect(
        loader_.displayComboBox, SIGNAL(currentIndexChanged(int)),
        loader_.stackedWidget, SLOT(setCurrentIndex(int)));


    QPixmap p((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/zoom_in.png")).c_str());
    zoom_in_cursor_ = QCursor(p.scaled(20,20));

    p=QPixmap((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/zoom_out.png")).c_str());
    zoom_out_cursor_ = QCursor(p.scaled(20,20));

    bool map_initialized_ = false;
    
   }

  /**
  @brief Default destructor
  @return void
  **/
  CMapConnector::~CMapConnector(void)
  {

  }

  /**
  @brief Resets map position to the loader
  @return void
  **/
  void CMapConnector::resetMap()
  {
    loader_.resetMap();
  }

  void CMapConnector::setMapXposition(double x)
  {
    loader_.setMapXposition(x);
  }

  void CMapConnector::setMapYposition(double y)
  {
    loader_.setMapYposition(y);
  }

  void CMapConnector::setMapRotation(int r)
  {
    loader_.setMapRotation(r);
  }

  void CMapConnector::setMapScale(double s)
  {
    loader_.setMapScale(s);
  }

  void CMapConnector::setMapTransparency(double t)
  {
    loader_.setMapTransparency(t);
    Q_EMIT mapTransparencyChanged(t);
  }

  void CMapConnector::displayMatchingImage(QImage* img)
  {
    loader_.displayMatchingImage(img);
  }
 
  void CMapConnector::displayMergedImage(QImage* img)
  {
    loader_.displayMergedImage(img);
  }


  /**
  @brief Returns the pos of the object to the scene
  @return QPointF : The pos to the scene
  **/
  QPointF CMapConnector::getPosition()
  {
    return loader_.getPosition();
  }
  /**
  @brief Returns the rotation (z-axis) of the object to the scene
  @return qreal : The rotation to the scene
  **/
  qreal CMapConnector::getRotation()
  {
    return loader_.getRotation();
  }

  /**
  @brief Returns the scaling of the object
  @return qreal : The scaling
  **/
  qreal CMapConnector::getScale()
  {
    return loader_.getScale();
  }

  /**
  @brief Returns the scaling of the slam Map
  @return qreal : The scaling
  **/
  qreal CMapConnector::getSlamMapScale()
  {
    return loader_.getSlamMapScale();
  }

  /**
  @brief Returns the scaling of the ground truth Map
  @return qreal : The scaling
  **/
  qreal CMapConnector::getGroundTruthMapScale()
  {
    return loader_.getGroundTruthMapScale();
  }

  /**
  @brief Returns the item's scene transformation matrix
  @return QTransform : The transformation matrix
  **/
  QTransform CMapConnector::getTransform()
  {
    return loader_.getTransform();
  }

  /**
  @brief Wrapper for the draw grid function of loader
  @param img [QImage*] The image on which the grid will be painted
  @param resolution [float] The map resolution
  @return void
  **/
  void CMapConnector::drawGrid(QImage *img,float resolution)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.drawGrid(img,resolution);
  }
  
  /**
  @brief Wrapper for the draw vertices function of loader
  @param img [QImage*] The image on which the vertices will be painted
  @param vertices [std::vector<geometry_msgs::Point>*] The voronoi vertices
  @return void
  **/
  void CMapConnector::drawVertices(QImage *img, std::vector<geometry_msgs::Point>* vertices, bool groundTruth)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.drawVertices(img, vertices, groundTruth);
  }
 
  /**
  @brief Wrapper for the draw voronoi function of loader
  @param img [QImage*] The image on which the voronoi will be painted
  @param voronoi [std::vector<geometry_msgs::Point>*] The voronoi diagram
  @return void
  **/
  void CMapConnector::drawVoronoi(QImage *img, std::vector<geometry_msgs::Point>* voronoi,bool groundTruth)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.drawVoronoi(img, voronoi, groundTruth);
  }
  /**
  @brief Wrapper for the draw voronoi function of loader
  @param img [QImage*] The image on which the voronoi will be painted
  @param voronoi [std::vector<geometry_msgs::Point>*] The voronoi diagram
  @return void
  **/
  void CMapConnector::drawMatchedVertices(QImage *img, std::vector<geometry_msgs::Point>* matchedVertices)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.drawMatchedVertices(img, matchedVertices);
  }


  /**
  @brief Emits the signalUpdateImage signal
  @param img [QImage*] The image to be updated
  @return void
  **/
  void CMapConnector::updateImage(QImage *img, bool groundTruth)
  {
    loader_.updateImage(img, groundTruth);
  }

  /**
  @brief Returns the CMapLoader object
  @return QWidget* : The object
  **/
  QWidget* CMapConnector::getLoader(void)
  {
    return static_cast<QWidget *>(&loader_);
  }

  /**
  @brief Sets the map initialization status
  @param mi [bool] The initialization status
  @return void
  **/
  void CMapConnector::setMapInitialized(bool mi)
  {
    map_initialized_ = mi;
  }
}
