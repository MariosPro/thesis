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
    slam_map->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    //slam_map->setAcceptHoverEvents(true);
    slam_map->setFocus();
    mapGraphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setScene(scene);
    mapGraphicsView->setInteractive(true);
    transparency = 0.5;
    offsetNotSet = true;
    //mapGraphicsView->show();
  }

  /**
  @brief Resets the map's position
  @return void
  **/
  void CMapLoader::resetMap()
  {
      slam_map->setPos(0 ,0);
      slam_map->setRotation(0);
      slam_map->setScale(1.0);
  }

  void CMapLoader::setMapXposition(double x)
  {
    slam_map->setPos(x, slam_map->y());

  }

  void CMapLoader::setMapYposition(double y)
  {
    slam_map->setPos(slam_map->x(), y);
  }

  void CMapLoader::setMapRotation(int r)
  {
    /*slam_map->setTransformOriginPoint(QPointF(slam_map->boundingRect().center()));*/
    /*slam_map->setRotation(r);*/
    slam_map->setMapRotation(r);
  }
 
  void CMapLoader::setMapScale(double s)
  {
/*    slam_map->setTransformOriginPoint(QPointF(slam_map->boundingRect().center()));*/
    /*slam_map->setScale(s);*/
    slam_map->setMapScale(s);
  }

  void CMapLoader::setMapTransparency(double t)
  {
    transparency = t;
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
      /*ROS_INFO_STREAM("INITIAL SCALING =" << (float)containerHeight / h );*/
      offsetScale = (float) containerHeight / h;
    }
    else
    {
      finalW = containerHeight * aspectRatio;
      finalH = containerHeight;
      /*ROS_INFO_STREAM("INITIAL SCALING =" << (float)containerWidth / w );*/
      offsetScale = (float) containerWidth / w;

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
    std::pair<int,int> newDims;
    /*newDims.first = img->width();*/
    /*newDims.second = img->height();*/
  /*  ROS_INFO_STREAM("container WIDTH=" << this->width());*/
    /*ROS_INFO_STREAM("container HEIGHT=" << this->height());*/

    if(groundTruth)
    {
     groundTruthMapWidth = img->width();
     groundTruthMapHeight =  img->height();
/*     ROS_INFO_STREAM("GROUND WIDTH=" << groundTruthMapWidth);*/
     /*ROS_INFO_STREAM("GROUND HEIGHT=" << groundTruthMapHeight);*/
     /*ROS_INFO_STREAM("ASPECT RATIO BEFORE=" << groundTruthMapWidth / (float)groundTruthMapHeight);*/

     std::pair<int,int> newDims = checkDimensions(img->width(), img->height(), 
                                                  this->width(), this->height());
 
  /*   ROS_INFO_STREAM("GROUND WIDTH AFTER=" <<  ground_truth_map->boundingRect().width()) ;*/
     /*ROS_INFO_STREAM("GROUND HEIGHT AFTER=" << ground_truth_map->boundingRect().height()) ;*/
     /*ROS_INFO_STREAM("ASPECT RATIO AFTER=" << ground_truth_map->boundingRect().width() / (float)ground_truth_map->boundingRect().height()) ;*/

      ground_truth_map->setPixmap(
        QPixmap().fromImage((
            *img).scaled(newDims.first,newDims.second,
                Qt::IgnoreAspectRatio,
                Qt::SmoothTransformation)));
      /*if(newDims.first > slam_map->boundingRect().width() && newDims.second > slam_map->boundingRect().height())*/
      {
        mapGraphicsView->resize(newDims.first, newDims.second);
        scene->setSceneRect(0, 0, newDims.first, newDims.second);
        mapGraphicsView->fitInView(scene->sceneRect());
      }
    }
      else
      {
        slamMapWidth = img->width();
        slamMapHeight =  img->height();
      /*  ROS_INFO_STREAM("SLAM WIDTH BEFORE=" << slamMapWidth);*/
        /*ROS_INFO_STREAM("SLAM HEIGHT BEFORE=" << slamMapHeight);*/
        /*ROS_INFO_STREAM("slam ASPECT RATIO BEFORE=" << slamMapWidth / (float) slamMapHeight);*/
        /*float sy = groundTruthMapWidth / (float)slamMapWidth;*/
        /*float sx = groundTruthMapHeight / (float)slamMapHeight;*/
 
        /*ROS_INFO_STREAM("SCALE BEFORE=" << sx << " " <<  sy);*/

        std::pair<int,int> newDims = checkDimensions(img->width(), img->height(), 
                                                 this->width(),this->height());
      
 /*       newDims.first = offsetScale * img->width();*/
        /*newDims.second = offsetScale * img->height();*/

        QPixmap pixmap = QPixmap::fromImage(
            (*img).scaled(newDims.first, newDims.second,
                Qt::IgnoreAspectRatio,
                Qt::SmoothTransformation));
        makeTransparent(&pixmap, transparency);
        slam_map->setPixmap(pixmap);

/*        if(offsetNotSet)*/
        /*{*/
          /*slam_map->setInitialMapScale(offsetScale);*/
          /*offsetNotSet = false;*/
        /*}*/

  /*      ROS_INFO_STREAM("OFFSET=" << offsetScale);*/
        /*ROS_INFO_STREAM("SLAM WIDTH =" <<  img->width());*/
        /*ROS_INFO_STREAM("SLAM HEIGHT =" << img->height());*/

      /*  ROS_INFO_STREAM("SLAM WIDTH AFTER=" <<  newDims.first);*/
        /*ROS_INFO_STREAM("SLAM HEIGHT AFTER=" << newDims.second);*/

 /*       ROS_INFO_STREAM("SLAM WIDTH AFTER=" <<  slam_map->boundingRect().width()) ;*/
        /*ROS_INFO_STREAM("SLAM HEIGHT AFTER=" << slam_map->boundingRect().height()) ;*/
        /*ROS_INFO_STREAM("SLAM ASPECT RATIO AFTER=" << slam_map->boundingRect().width() / (float)slam_map->boundingRect().height());*/
        /*float ny = ground_truth_map->boundingRect().width() / (float) slam_map->boundingRect().width();*/
        /*float nx = ground_truth_map->boundingRect().height() / (float) slam_map->boundingRect().height();*/
        /*ROS_INFO_STREAM("SCALE AFTER=" << nx << " " <<  ny);*/


      if(newDims.first > ground_truth_map->boundingRect().width() && newDims.second > ground_truth_map->boundingRect().height())
      {
      mapGraphicsView->resize(newDims.first, newDims.second);
      scene->setSceneRect(0, 0, newDims.first, newDims.second);
      mapGraphicsView->fitInView(scene->sceneRect());
      }
    }
 
  }//.scaled(newDims.first, newDims.second,


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
    QPointF p(0,0);
    if (slam_map->translated)
      return slam_map->scenePos();
    else 
      return p;
  }
  
  /**
  @brief Returns the rotation (z-axis) of the object to the scene
  @return qreal : The rotation to the scene
  **/
  qreal CMapLoader::getRotation()
  {
    if(slam_map->rotated)
      return slam_map->rotation();
    else
      return 0;
  }
  
  /**
  @brief Returns the scaling of the object
  @return qreal : The scaling
  **/
  qreal CMapLoader::getScale()
  {
    if (slam_map->scaled)
      return slam_map->scale();
    else
      return 1;
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
