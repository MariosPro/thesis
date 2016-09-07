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
/*  enum EOGMMapState*/
  //{
    //NORMAL,
    //ZOOMIN,
    //ZOOMOUT,
   /*};*/

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
      //EOGMMapState map_state_;

      //!< Object of CMapLoader
      CMapLoader loader_;

      //!< True if map is initialized
      bool map_initialized_;

      bool translated;

      bool rotated;

      bool scaled;

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
 
      void setMapXposition(double x);

      void setMapYposition(double y);

      void setMapRotation(int r);

      void setMapScale(double s);

      void setMapTransparency(double t);

      void displayMatchingImage(QImage* img);

      void displayMergedImage(QImage* img);
      /**
      @brief Returns the pos of the object to the scene
      @return QPointF : The pos to the scene
      **/
      QPointF getPosition();

      /**
      @brief Returns the rotation (z-axis) of the object to the scene
      @return qreal : The rotation to the scene
      **/
      qreal getRotation();

      /**
      @brief Returns the scaling of the object
      @return qreal : The scaling
      **/
      qreal getScale();

      /**
      @brief Returns the scaling of the slam Map
      @return qreal : The scaling
      **/
      qreal getSlamMapScale();

      /**
      @brief Returns the scaling of the ground Truth Map
      @return qreal : The scaling
      **/
      qreal getGroundTruthMapScale();

      /**
      @brief Returns the item's scene transformation matrix
      @return QTransform : The transformation matrix
      **/
      QTransform getTransform();

      /**
      @brief Emits the signalUpdateImage signal
      @param img [QImage*] The image to be updated
      @return void
      **/
      void updateImage(QImage *img, bool groundTruth);

      /**
      @brief Sets the map initialization status
      @param mi [bool] The initialization status
      @return void
      **/
      void setMapInitialized(bool mi);

      /**
      @brief Wrapper for the draw grid function of loader
      @param img [QImage*] The image on which the grid will be painted
      @param resolution [float] The map resolution
      @return void
      **/
      void drawGrid(QImage *img,float resolution);
      
      /**
      @brief Wrapper for the draw vertices function of loader
      @param img [QImage*] The image on which the vertices will be painted
      @param vertices [std::vector<geometry_msgs::Point>*] The voronoi vertices
      @return void
      **/
      void drawVertices(QImage *img, std::vector<geometry_msgs::Point>* vertices, bool groundTruth);
 
      /**
      @brief Wrapper for the draw voronoi function of loader
      @param img [QImage*] The image on which the voronoi will be painted
      @param vertices [std::vector<geometry_msgs::Point>*] The voronoi diagram
      @return void
      **/
      void drawVoronoi(QImage *img, std::vector<geometry_msgs::Point>* voronoi, bool groundTruth);
 
      /**
      @brief Wrapper for the draw voronoi function of loader
      @param img [QImage*] The image on which the voronoi will be painted
      @param vertices [std::vector<geometry_msgs::Point>*] The voronoi diagram
      @return void
      **/
      void drawMatchedVertices(QImage *img, std::vector<geometry_msgs::Point>* matchedVertices,
                               bool groundTruth);

      /**
      @brief Returns the CMapLoader object
      @return QWidget* : The object
      **/
      QWidget* getLoader(void);

      /**
      @brief Resets map position to the loader
      @return void
      **/
      void resetMap();

    //------------------------------------------------------------------------//
    public Q_SLOTS:
    
      //------------------------------------------------------------------------//
    Q_SIGNALS:
      void mapPosChanged(qreal x, qreal y);

      void mapRotationChanged(qreal r);

      void mapScaleChanged(qreal s);

      void mapTransparencyChanged(qreal t);

  };
}

#endif
