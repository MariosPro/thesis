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

#include "ogm_gui/ogm_map_loader.h"

namespace ogm_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CMapLoader::CMapLoader(int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    scene = new QGraphicsScene(mapGraphicsView);
    ground_truth_map = new CMapItem();
    slam_map = new CMapItem();
    scene->addItem(ground_truth_map);
    scene->addItem(slam_map);
    slam_map->setFlag(QGraphicsItem::ItemIsMovable);
    slam_map->setFlag(QGraphicsItem::ItemIsFocusable);
    slam_map->setFocus();
    mapGraphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setScene(scene);
    mapGraphicsView->setInteractive(true);
    //mapGraphicsView->show();
  }

  /**
  @brief Resets the map's position
  @return void
  **/
  void CMapLoader::resetMap()
  {
      ROS_INFO_STREAM("SET POS");
      slam_map->setPos(0 ,0);
      slam_map->setRotation(0);
      slam_map->setScale(1.0);
  }

  /**
  @brief Return the dimensions according to the container size
  @param w [int] Image width
  @param h [int] Image height
  @return std::pair<int,int> : The size the map must be resized to
  **/
  std::pair<int,int> CMapLoader::checkDimensions(int w, int h, 
                      int containerWidth, int containerHeight)
  {
    /*float containerWidth = this->width();*/
    /*float containerHeight = this->height();*/
    float aspectRatio = (float)w / (float)h;
    float finalW,finalH;
    if(containerHeight * aspectRatio > containerWidth)
    {
      finalW = containerWidth;
      finalH = containerWidth / aspectRatio;
    }
    else
    {
      finalW = containerHeight * aspectRatio;
      finalH = containerHeight;
    }
    return std::pair<int,int>(finalW,finalH);
  }
  
  /**
  @brief Updates the image
  @param img [QImage*] The image to be updated
  @return void
  **/
  void CMapLoader::updateImage(QImage *img, bool groundTruth)
  {
    //internal_img_ = img;
    if(groundTruth)
    {
     std::pair<int,int> newDims = checkDimensions(img->width(), img->height(), 
                                                 this->width(), this->height());
      ground_truth_map->setPixmap(
        QPixmap().fromImage((
            *img).scaled(newDims.first,newDims.second,
                Qt::IgnoreAspectRatio,
                Qt::SmoothTransformation)));
      mapGraphicsView->resize(newDims.first, newDims.second);
      scene->setSceneRect(0, 0, newDims.first, newDims.second);
      mapGraphicsView->fitInView(scene->sceneRect());
    }
      else
      {
        std::pair<int,int> newDims = checkDimensions(img->width(), img->height(), 
                                                 ground_truth_map->boundingRect().width(),
                                                 ground_truth_map->boundingRect().height());
        QPixmap pixmap = QPixmap::fromImage(
              (*img).scaled(newDims.first, newDims.second,
                  Qt::IgnoreAspectRatio,
                  Qt::SmoothTransformation));
        makeTransparent(&pixmap, 0.5);
        slam_map->setPixmap(pixmap);

      /*mapGraphicsView->resize(newDims.first, newDims.second);*/
      //scene->setSceneRect(0, 0, newDims.first, newDims.second);
      /*mapGraphicsView->fitInView(scene->sceneRect());*/
    }
  }

  /**
  @brief Makes the image transparent
  @param img [QImage*] The image to be made transparent
  @param opacity [double] The map opacity
  @return void
  **/
  void CMapLoader::makeTransparent(QPixmap *img, double opacity)  
  {
    QImage image(img->size(), QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    QPainter p(&image);
    p.setOpacity(opacity);
    p.drawPixmap(0, 0, *img);
    p.end();
    *img = QPixmap::fromImage(image);
  }

  /**
  @brief Draws a grid in an image
  @param img [QImage*] The image for the grid to be drawn on
  @param resolution [float] The map resolution
  @return void
  **/
  void CMapLoader::drawGrid(QImage *img,float resolution)
  {
    QPainter painter(img);
    painter.setPen(QColor(100, 100, 100, 150));
    int pix = 1.0 / resolution;
    for(unsigned int i = 1 ; i <= img->width(); i++)
    {
      painter.drawLine(0, i * pix, img->width() - 1, i * pix);
    }
    for(unsigned int i = 1 ; i <= img->height(); i++)
    {
      painter.drawLine(i * pix, 0, i * pix, img->height() - 1);
    }
  }

  /**
  @brief Returns the pos of the object to the scene
  @return QPointF : The pos to the scene
  **/
  QPointF CMapLoader::getPosition()
  {
    return slam_map->scenePos();
  }
  
  /**
  @brief Returns the rotation (z-axis) of the object to the scene
  @return qreal : The rotation to the scene
  **/
  qreal CMapLoader::getRotation()
  {
    return slam_map->rotation();
  }
  
  /**
  @brief Returns the scaling of the object
  @return qreal : The scaling
  **/
  qreal CMapLoader::getScale()
  {
    return slam_map->scale();
  }

  /**
  @brief Returns the item's scene transformation matrix
  @return QTransform : The transformation matrix
  **/
  QTransform CMapLoader::getTransform()
  {
    return slam_map->sceneTransform();
  }

}
