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

#include "ogm_gui/ogm_map_item.h"

namespace ogm_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CMapItem::CMapItem()
  {
    factor = 1.05;
  }

  /**
  @brief Captures the keypress event
  @param event [QKeyEvent*] The key event
  @return void
  **/
  void CMapItem::keyPressEvent(QKeyEvent *event)
  {

    if (event->key() == Qt::Key_Left)
    {
      setPos(x() - 1, y());
      Q_EMIT posChanged(x(), y());
    }
    else if (event->key() == Qt::Key_Right)
    {
      setPos(x() + 1, y());
      Q_EMIT posChanged(x(), y());
    }
    else if (event->key() == Qt::Key_Up)
    {
      setPos(x(), y() - 1);
      Q_EMIT posChanged(x(), y());
    }
    else if (event->key() == Qt::Key_Down)
    {
      setPos(x(), y() + 1);
      Q_EMIT posChanged(x(), y());
    }
    else if (event->key() == Qt::Key_E)
    {
      setTransformOriginPoint(QPointF(this->boundingRect().center()));
      setRotation(rotation() - 1);
      Q_EMIT rotationChanged(rotation());

    }
    else if (event->key() == Qt::Key_R)
    {
      setTransformOriginPoint(QPointF(this->boundingRect().center()));
      setRotation(rotation() + 1);
      Q_EMIT rotationChanged(rotation());
    }
    else if (event->key() == Qt::Key_Plus)
    {
      setTransformOriginPoint(QPointF(this->boundingRect().center()));
      setScale(scale() * factor);
      Q_EMIT scaleChanged(scale());
    }
    else if (event->key() == Qt::Key_Minus)
    {
      setTransformOriginPoint(QPointF(this->boundingRect().center()));
      setScale(scale() / factor);
      Q_EMIT scaleChanged(scale());
    }
  }
 }


