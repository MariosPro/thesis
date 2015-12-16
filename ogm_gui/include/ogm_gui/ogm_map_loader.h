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

#ifndef OGM_MAP_LOADER
#define OGM_MAP_LOADER

#include <QDebug>
#include "ui_map.h"
#include "ogm_gui/ogm_map_item.h"
#include "ogm_gui/ogm_tools.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/ 
namespace ogm_gui
{
  /**
  @class CMapLoader
  @brief Implements the low level Qt functionalities of the map widget. Inherits from Ui_mapWidget (generated from an ui file) and QWidget. 
  **/ 
  class CMapLoader : 
    public QWidget, 
    public Ui_mapWidget
  {
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;

      //!< The scene that is used to add the maps
      QGraphicsScene* scene;

      //!< Internal image used before a map is loaded
      QImage*  internal_img_;

      double transparency;

      int groundTruthMapWidth;

      int groundTruthMapHeight;

      int slamMapWidth;

      int slamMapHeight;

      float offsetScale;

      bool offsetNotSet;

      /**
      @brief Return the dimensions according to the container size
      @param w [int] Image width
      @param h [int] Image height
      @param containerWidth [int] The container width
      @param containerHeight [int] The container height
      @return std::pair<int,int> : The size the map must be resized to
      **/
      std::pair<int,int> checkDimensions(int w, int h,
                                         int containerWidth,
                                         int containerHeight);
      
    //------------------------------------------------------------------------//  
    public:
      
      //!< the pixmap that is used to display the ground thuth map
      CMapItem* ground_truth_map;

      //!< the pixmap that is used to display the slam-produced map
      CMapItem* slam_map;

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CMapLoader(int argc, char **argv);
      
      /**
      @brief Resets the map's position
      @return void
      **/
      void resetMap();
       
      /**
      @brief Updates the image
      @param img [QImage*] The image to be updated
      @return void
      **/
      void updateImage(QImage *img, bool groundTruth);
      
      /**
      @brief Draws a grid in an image
      @param img [QImage*] The image for the grid to be drawn on
      @param resolution [float] The map resolution
      @return void
      **/
      void drawGrid(QImage *img,float resolution);
      
      void setMapXposition(double x);

      void setMapYposition(double y);

      void setMapRotation(int r);

      void setMapScale(double s);

      void setMapTransparency(double t);

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
      @brief Returns the item's scene transformation matrix
      @return QTransform : The transformation matrix
      **/
      QTransform getTransform();

      /**
      @brief Makes the image transparent
      @param img [QImage*] The image to be made transparent
      @param opacity [double] The map opacity
      @return void
      **/
      void makeTransparent(QPixmap *img, double opacity);
   };
}

#endif
