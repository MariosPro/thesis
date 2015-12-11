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

  QVariant CMapItem::itemChange(GraphicsItemChange change, const QVariant & value)
  {
    if (change == ItemPositionChange && scene()) 
    {
      setPos( x(),  y());
      Q_EMIT posChanged(x(), y());
    }

    return QGraphicsItem::itemChange(change, value);
  }

  void CMapItem::setMapRotation(int r)
  {
    setTransformOriginPoint(QPointF(this->boundingRect().center()));
    setRotation(r);
    QPointF sp = scenePos();
    Q_EMIT posChanged(sp.x(), sp.y());
  }

  void CMapItem::setMapScale(double s)
  {
    setTransformOriginPoint(QPointF(this->boundingRect().center()));
    setScale(s);
    QPointF  sp = scenePos();
    Q_EMIT posChanged(sp.x(), sp.y());
    Q_EMIT rotationChanged(rotation());
  }

  /**
  @brief Captures the keypress event
  @param event [QKeyEvent*] The key event
  @return void
  **/
  void CMapItem::keyPressEvent(QKeyEvent *event)
  {

    setTransformOriginPoint(QPointF(this->boundingRect().center()));
    QPointF sp;
    if (event->key() == Qt::Key_Left)
    {
      setPos(x() - 1, y());
      sp = scenePos();
      Q_EMIT posChanged(x(), y());
      /*qDebug() << "pos=" << x() <<" " <<y();*/
      /*qDebug() << "scenePos=" << scenePos();*/
    }
    else if (event->key() == Qt::Key_Right)
    {
      setPos(x() + 1, y());
      sp = scenePos();
      Q_EMIT posChanged(x(), y());
      /*qDebug() << "pos=" << x() <<" " <<y();*/
      /*qDebug() << "scenePos=" << scenePos();*/
    }
    else if (event->key() == Qt::Key_Up)
    {
      setPos(x(), y() - 1);
      sp = scenePos();
      Q_EMIT posChanged(x(), y());
     /* qDebug() << "pos=" << x() <<" " <<y();*/
      /*qDebug() << "scenePos=" << scenePos();*/
    }
    else if (event->key() == Qt::Key_Down)
    {
      setPos(x(), y() + 1);
      sp = scenePos();
      Q_EMIT posChanged(x(), y());
  /*    qDebug() << "pos=" << x() <<" " << y();*/
      /*qDebug() << "scenePos=" << scenePos();*/
    }
    else if (event->key() == Qt::Key_E)
    {
      setRotation(rotation() - 1);
      sp = scenePos();
      Q_EMIT rotationChanged(rotation());
      Q_EMIT posChanged(sp.x(), sp.y());
/*      qDebug() << "pos=" << x() <<" " <<y() << rotation();*/
      /*qDebug() << "scenePos=" << scenePos();*/
    }
    else if (event->key() == Qt::Key_R)
    {
      setRotation(rotation() + 1);
      sp = scenePos();
      Q_EMIT rotationChanged(rotation());
      Q_EMIT posChanged(sp.x(), sp.y());
  /*    qDebug() << "pos=" << x() <<" " <<y();*/
      //qDebug() << "scenePos=" << scenePos();
    }
    else if (event->key() == Qt::Key_Plus)
    {
      setScale(scale() * factor);
      sp = scenePos();
      Q_EMIT scaleChanged(scale());
      Q_EMIT posChanged(sp.x(), sp.y());
      Q_EMIT rotationChanged(rotation());
      //qDebug() << "pos=" << x() <<" " <<y();
      //qDebug() << "scenePos=" << scenePos() << sceneTransform();
    }

    else if (event->key() == Qt::Key_Minus)
    {
      setScale(scale() / factor);
      sp = scenePos();
      Q_EMIT scaleChanged(scale());
      Q_EMIT posChanged(sp.x(), sp.y());
      Q_EMIT rotationChanged(rotation());
    /*  qDebug() << "pos=" << x() <<" " <<y();*/
      /*qDebug() << "scenePos=" << scenePos() << sceneTransform();*/
    }
  }
 }


