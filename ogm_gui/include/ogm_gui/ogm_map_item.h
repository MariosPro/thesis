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

#ifndef OGM_MAP_ITEM
#define OGM_MAP_ITEM

#include <QtGui/QKeyEvent>
#include <QtCore/QObject>
#include <QtWidgets/QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include "ogm_gui/ogm_tools.h"


/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class CMapItem
  @brief Implements the QGraphicsPixmapItem for map display.
  **/ 
  class CMapItem : public QObject, public QGraphicsPixmapItem 
  {
    Q_OBJECT 
    //------------------------------------------------------------------------//
    private:
           qreal factor;
           qreal xCoord;
           qreal yCoord;
           qreal rot;

    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @return void
      **/
      CMapItem();

  /*    void mousePressEvent(QGraphicsSceneMouseEvent *event);*/

      /*void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);*/

      void setMapRotation(int r);
      void setMapScale(double s);
      /**
      @brief Captures the keypress event
      @param event [QKeyEvent*] The key event
      @return void
      **/
      void keyPressEvent(QKeyEvent *event);

    public Q_SLOTS:

    Q_SIGNALS:

    void posChanged(qreal x, qreal y);

    void rotationChanged(qreal r);

    void scaleChanged(qreal s);
  };
}

#endif
