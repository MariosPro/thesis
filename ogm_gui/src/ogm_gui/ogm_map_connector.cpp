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
    map_state_ = NORMAL;

    loader_.map->setScaledContents(true);

    loader_.map->installEventFilter(this);

    QObject::connect(
      this,SIGNAL(signalUpdateImage(QImage *)),
      this, SLOT(serveImage(QImage *)));

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
  @brief Sets map initial size to the loader
  @param s [QSize] Map size
  @return void
  **/
  void CMapConnector::setInitialImageSize(QSize s)
  {
    loader_.setInitialImageSize(s);
  }

  /**
  @brief Updates the map zoom. Wrapper for a loader function
  @param p [QPoint] The point of the zoom event
  @param z [bool] True if zoom in, false if zoom out
  @return void
  **/
  void CMapConnector::updateZoom(QPoint p,bool z)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.updateZoom(p,z);
  }

  /**
  @brief Updates the map center
  @param p [QPoint] The new center
  @return void
  **/
  void CMapConnector::updateCenter(QPoint p){
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.updateCenter(p);
  }

  /**
  @brief Returns the point in the real map image. Wrapper for a loader function.
  @param p [QPoint] A point
  @return QPoint : The "real" point in the original map
  **/
  QPoint CMapConnector::getGlobalPoint(QPoint p)
  {
    return loader_.getGlobalPoint(p);
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
  @brief General event filter. Captures all events
  @param watched [QObject*] The object in which the event was triggered
  @param event [QEvent*] The type of event
  @return bool : True is event served
  **/
  bool CMapConnector::eventFilter( QObject* watched, QEvent* event ) 
  {
    if ( ! map_initialized_ )
    {
      return false;
    }
    if(watched == loader_.map)
    {
      if(event->type() == QEvent::MouseButtonPress)
      {

        loader_.map->setFocus(Qt::MouseFocusReason);

        const QMouseEvent* const me = 
          static_cast<const QMouseEvent*>( event );
        QPoint p = me->pos();
        if(me->button() == Qt::RightButton)
        {
          map_state_ = NORMAL;
          loader_.map->setCursor(QCursor(Qt::CrossCursor));
          Q_EMIT itemClicked(p,Qt::RightButton);
        }
        else if(me->button() == Qt::LeftButton)
        {
          if(map_state_ == ZOOMIN)
          {
            Q_EMIT zoomInPressed(p);
          }
          else if(map_state_ == ZOOMOUT)
          {
            Q_EMIT zoomOutPressed(p);
          }
          else if(map_state_ == NORMAL)
          {
            Q_EMIT itemClicked(p,Qt::LeftButton);
          }
        }
      }
      else if(event->type() == QEvent::Wheel)
      {
        const QWheelEvent* const me = 
          static_cast<const QWheelEvent*>( event );
        QPoint p = me->pos();
        if(me->delta() > 0)
        {
          Q_EMIT zoomInPressed(p);
        }
        else
        {
          Q_EMIT zoomOutPressed(p);
        }
      }
      else if(event->type() == QEvent::KeyPress)
      {
        const QKeyEvent* const me =
          static_cast<const QKeyEvent*>( event );

        if ( ! map_initialized_ )
        {
          return false;
        }
        if(me->key() == Qt::Key_Right)
        {
          loader_.moveDirectionally(Qt::Key_Right);
        }
        else if(me->key() == Qt::Key_Left)
        {
          loader_.moveDirectionally(Qt::Key_Left);
        }
        else if(me->key() == Qt::Key_Up)
        {
          loader_.moveDirectionally(Qt::Key_Up);
        }
        else if(me->key() == Qt::Key_Down)
        {
          loader_.moveDirectionally(Qt::Key_Down);
        }
      }
    }
    return false;
  }

  /**
  @brief Emits the signalUpdateImage signal
  @param img [QImage*] The image to be updated
  @return void
  **/
  void CMapConnector::updateImage(QImage *img)
  {
    Q_EMIT signalUpdateImage(img);
  }

  /**
  @brief Called from signalUpdateImage signal. Calls the updateImage of CMapLoader
  @param img [QImage*] The image to be painted
  @return void
  **/
  void CMapConnector::serveImage(QImage *img)
  {
    loader_.updateImage(img);
  }

  /**
  @brief Calls the Qt function that gets the real point that the event happened
  @param p [QPoint] The point of the event
  @return QPoint : The "real" point
  **/
  QPoint CMapConnector::mapToGlobal(QPoint p)
  {
    return loader_.mapToGlobal(p);
  }

  /**
  @brief Called when zoom in event happens
  @param state [bool] True when zoom in active
  @return void
  **/
  void CMapConnector::setCursorZoomIn(bool state)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    if(state)
    {
      map_state_ = ZOOMIN;
      loader_.map->setCursor(zoom_in_cursor_);
    }
    else
    {
      map_state_ = NORMAL;
      loader_.map->setCursor(QCursor(Qt::CrossCursor));
    }
  }

  /**
  @brief Called when zoom out event happens
  @param state [bool] True when zoom out active
  @return void
  **/
  void CMapConnector::setCursorZoomOut(bool state)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    if(state)
    {
      map_state_ = ZOOMOUT;
      loader_.map->setCursor(zoom_out_cursor_);
    }
    else
    {
      map_state_ = NORMAL;
      loader_.map->setCursor(QCursor(Qt::CrossCursor));
    }
  }

  /**
  @brief Called when zoom adjusted event happens
  @param state [bool] True when zoom adjusted active
  @return void
  **/
  void CMapConnector::setCursorAdjusted(bool state)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    loader_.resetZoom();
    map_state_ = NORMAL;
    loader_.map->setCursor(QCursor(Qt::CrossCursor));
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

  /**
  @brief Called when moveUpMap signal is received.Moves the map on upward direction
  @return void
  **/
  void CMapConnector::moveUp(void)
  {

  }

  /**
  @brief Called when moveDownMap signal is received.Moves the map on downward direction
  @return void
  **/
  void CMapConnector::moveDown(void)
  {

  }

  /**
  @brief Called when moveLeftMap signal is received.Moves the map on leftward direction
  @return void
  **/
  void CMapConnector::moveLeft(void)
  {

  }

  /**
  @brief Called when moveRightMap signal is received.Moves the map on rightward direction
  @return void
  **/
  void CMapConnector::moveRight(void)
  {

  }

  /**
  @brief Called when rotateLeftMap signal is received.Rotates the map on leftward direction
  @return void
  **/
  void CMapConnector::rotateLeft(void)
  {

  }

  /**
  @brief Called when rotateRightMap signal is received.Rotates the map on rightward direction
  @return void
  **/
  void CMapConnector::rotateRight(void)
  {

  }

  /**
  @brief Called when scaleMap signal is received.Scales the map
  @return void
  **/
  void CMapConnector::scale(void)
  {

  }

}
