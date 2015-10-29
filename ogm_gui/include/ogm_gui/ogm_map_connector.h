/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2015 OCM Validator
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

#ifndef OGM_MAP_CONNECTOR
#define OGM_MAP_CONNECTOR

#include "ogm_gui/ogm_map_loader.h"

/**
@namespace ogm_gui

@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @enum EOGMState
  @brief Holds the possible map states
  **/
  enum EOGMMapState
  {
    NORMAL,
    ZOOMIN,
    ZOOMOUT,
   };

  /**
  @class CMapConnector
  @brief Serves the Qt events of the map-holding widget. Inherits from QObject
  **/
  class CMapConnector :
    public QObject
  {
    Q_OBJECT
    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int argc_;
      //!< Input arguments
      char** argv_;

      //!< Mouse cursor for zoom in
      QCursor zoom_in_cursor_;
      //!< Mouse cursor for zoom out
      QCursor zoom_out_cursor_;

      //!< Holds the map state
      EOGMMapState map_state_;

      //!< Object of CMapLoader
      CMapLoader loader_;

      //!< True if map is initialized
      bool map_initialized_;

    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CMapConnector(int argc, char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CMapConnector(void);

      /**
      @brief Emits the signalUpdateImage signal
      @param img [QImage*] The image to be updated
      @return void
      **/
      void updateImage(QImage *img);

      /**
      @brief Sets map initial size to the loader
      @param s [QSize] Map size
      @return void
      **/
      void setInitialImageSize(QSize s);

      /**
      @brief Updates the map zoom. Wrapper for a loader function
      @param p [QPoint] The point of the zoom event
      @param z [bool] True if zoom in, false if zoom out
      @return void
      **/
      void updateZoom(QPoint p,bool z);

      /**
      @brief Updates the map center
      @param p [QPoint] The new center
      @return void
      **/
      void updateCenter(QPoint p);

      /**
      @brief Returns the point in the real map image. Wrapper for a loader function.
      @param p [QPoint] A point
      @return QPoint : The "real" point in the original map
      **/
      QPoint getGlobalPoint(QPoint p);

      /**
      @brief Wrapper for the draw grid function of loader
      @param img [QImage*] The image on which the grid will be painted
      @param resolution [float] The map resolution
      @return void
      **/
      void drawGrid(QImage *img,float resolution);

      /**
      @brief Calls the Qt function that gets the real point that the event happened
      @param p [QPoint] The point of the event
      @return QPoint : The "real" point
      **/
      QPoint mapToGlobal(QPoint p);

      /**
      @brief Returns the CMapLoader object
      @return QWidget* : The object
      **/
      QWidget* getLoader(void);

      /**
      @brief Sets the map initialization status
      @param mi [bool] The initialization status
      @return void
      **/
      void setMapInitialized(bool mi);

    //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief General event filter. Captures all events
      @param watched [QObject*] The object in which the event was triggered
      @param event [QEvent*] The type of event
      @return bool : True is event served
      **/
      bool eventFilter( QObject* watched, QEvent* event);

      /**
      @brief Called from signalUpdateImage signal. Calls the updateImage of CMapLoader
      @param img [QImage*] The image to be painted
      @return void
      **/
      void serveImage(QImage *img);

      /**
      @brief Called when zoom in event happens
      @param state [bool] True when zoom in active
      @return void
      **/
      void setCursorZoomIn(bool state);

      /**
      @brief Called when zoom out event happens
      @param state [bool] True when zoom out active
      @return void
      **/
     void setCursorZoomOut(bool state);

      /**
      @brief Called when zoom adjusted event happens
      @param state [bool] True when zoom adjusted active
      @return void
      **/
      void setCursorAdjusted(bool state);

      /**
      @brief Called when moveUpMap signal is received. Moves the map on upward direction
      @return void
      **/
      void moveUp(void);

      /**
      @brief Called when moveDownMap signal is received. Moves the map on downard direction
      @return void
      **/
      void moveDown(void);

      /**
      @brief Called when moveLeftMap signal is received. Moves the map on leftward direction
      @return void
      **/
      void moveLeft(void);

      /**
      @brief Called when moveRightMap signal is received. Moves the map on rightward direction
      @return void
      **/
      void moveRight(void);

      /**
      @brief Called when rotateLeftMap signal is received. Rotates the map on leftward direction
      @return void
      **/
      void rotateLeft(void);

      /**
      @brief Called when rotateRightMap signal is received. Moves the map on rightward direction
      @return void
      **/
      void rotateRight(void);

      /**
      @brief Called when scaleMap signal is received. Scales the map.
      @return void
      **/
      void scale(void);

    //------------------------------------------------------------------------//
    Q_SIGNALS:

      /**
      @brief Emmited in updateImage function
      @param img [QImage*] The image to be updated
      @return void
      **/
      void signalUpdateImage(QImage *img);

      /**
      @brief Emmited when click is captured and state is ZOOMIN
      @param p [QPoint] The event point
      @return void
      **/
      void zoomInPressed(QPoint p);

      /**
      @brief Emmited when click is captured and state is ZOOMOUT
      @param p [QPoint] The event point
      @return void
      **/
      void zoomOutPressed(QPoint p);

      /**
      @brief Emmited when click is captured and state is NORMAL
      @param p [QPoint] The event point
      @param b [Qt::MouseButton] The mouse button clicked
      @return void
      **/
      void itemClicked(QPoint p,Qt::MouseButton b);
 };
}

#endif
