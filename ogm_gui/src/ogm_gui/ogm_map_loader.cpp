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
    scene->addItem(slam_map);
    scene->addItem(ground_truth_map);
    scene->setSceneRect(0, 0, this->stackedWidget->width(), this->stackedWidget->height() - 20);
    ground_truth_map->setFlag(QGraphicsItem::ItemIsMovable);
    ground_truth_map->setFlag(QGraphicsItem::ItemIsFocusable);
    ground_truth_map->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    ground_truth_map->setFocus();
    mapGraphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapGraphicsView->setScene(scene);
    mapGraphicsView->fitInView(scene->sceneRect());
    mapGraphicsView->setInteractive(true);
    transparency = 0.5;
    _initialMatched = false;
    _finalMatched = false;
    _merged = false;
    //mapGraphicsView->show();
  }

  /**
  @brief Resets the map's position
  @return void
  **/
  void CMapLoader::resetMap()
  {
    ground_truth_map->setPos(0 ,0);
    ground_truth_map->setRotation(0);
    ground_truth_map->setScale(1.0);
    //slamOffsetScale = 1.0;
    //groundTruthOffsetScale = 1.0;
  }

  void CMapLoader::setMapXposition(double x)
  {
    ground_truth_map->setPos(x, ground_truth_map->y());

  }

  void CMapLoader::setMapYposition(double y)
  {
    ground_truth_map->setPos(ground_truth_map->x(), y);
  }

  void CMapLoader::setMapRotation(int r)
  {
    /*ground_truth_map->setTransformOriginPoint(QPointF(slam_map->boundingRect().center()));*/
    /*ground_truth_map->setRotation(r);*/
    ground_truth_map->setMapRotation(r);
  }
 
  void CMapLoader::setMapScale(double s)
  {
/*    ground_truth_map->setTransformOriginPoint(QPointF(slam_map->boundingRect().center()));*/
    /*ground_truth_map->setScale(s);*/
    ground_truth_map->setMapScale(s);
  }

  void CMapLoader::setMapTransparency(double t)
  {
    transparency = t;
  }

  void CMapLoader::displayInitialMatchingImage(QImage* img)
  {
   if(!img->isNull())
   {
      newDims = checkDimensions(img->width(), img->height(),
                               this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

       initialMatchingLabel->setPixmap(QPixmap::fromImage(*img).scaled(newDims.first,newDims.second,
                Qt::IgnoreAspectRatio,
                Qt::SmoothTransformation));
      initialMatchingLabel->resize(newDims.first, newDims.second);

      _initialMatched = true;
    }
  }
  
  void CMapLoader::displayFinalMatchingImage(QImage* img)
  {
   if(!img->isNull())
   {
      newDims = checkDimensions(img->width(), img->height(),
                               this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

       finalMatchingLabel->setPixmap(QPixmap::fromImage(*img).scaled(newDims.first,newDims.second,
                Qt::IgnoreAspectRatio,
                Qt::SmoothTransformation));
      finalMatchingLabel->resize(newDims.first, newDims.second);

      _finalMatched = true;
    }
  }

  void CMapLoader::displayMergedImage(QImage* img)
  {
   if(!img->isNull())
   {

     newDims = checkDimensions(img->width(), img->height(),
                                 this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

      mergedLabel->setPixmap(QPixmap::fromImage(*img).scaled(newDims.first,newDims.second,
                  Qt::IgnoreAspectRatio,
                  Qt::SmoothTransformation));
      mergedLabel->resize(newDims.first, newDims.second);
      
      _merged = true;
    }
  }

  /**
  @brief Return the dimensions according to the container size
  @param w [int] Image width
  @param h [int] Image height
  @return std::pair<int,int> : The size the map must be resized to
  **/
  std::pair<int,int> CMapLoader::checkDimensions(int w, int h, 
                      int containerWidth, int containerHeight, bool groundTruth)
  {

    double aspectRatio = (double)w / (double)h;
    double finalW,finalH;

    if(containerHeight * aspectRatio > containerWidth)
    {
      finalW = containerWidth;
      finalH = containerWidth / aspectRatio;

      if(groundTruth)
        groundTruthOffsetScale = (double) containerWidth / (double)w;
      else
        slamOffsetScale = (double) containerWidth / (double)w;
    }
    else
    {
      finalW = containerHeight * aspectRatio;
      finalH = containerHeight;

      if(groundTruth)
        groundTruthOffsetScale = (double) containerHeight / (double)h;
      else
        slamOffsetScale = (double) containerHeight / (double)h;

    }
    groundTruthOffsetScale = round(groundTruthOffsetScale * 100) / 100;
    slamOffsetScale = round(slamOffsetScale * 100) / 100;

    return std::pair<int,int>(finalW,finalH);
  }

  /**
  @brief Updates the image
  @param img [QImage*] The image to be updated
  @return void
  **/
  void CMapLoader::updateImage(QImage *img, bool groundTruth)
  {
    if(!groundTruth)
    {

      newDims = checkDimensions(img->width(), img->height(),
                               this->stackedWidget->width(), this->stackedWidget->height() - 20, false);
      slam_map->setPixmap(
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
      newDims = checkDimensions(img->width(), img->height(),
                                slam_map->boundingRect().width(),
                                slam_map->boundingRect().height(), true);

      QPixmap pixmap = QPixmap::fromImage(
          (*img).scaled(newDims.first, newDims.second,
              Qt::IgnoreAspectRatio,
              Qt::SmoothTransformation));
      makeTransparent(&pixmap, transparency);
      ground_truth_map->setPixmap(pixmap);
    }
    
    if(_initialMatched)
    {
      newDims = checkDimensions(initialMatchingLabel->pixmap()->width(), initialMatchingLabel->pixmap()->height(),
                             this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

      initialMatchingLabel->setPixmap(initialMatchingLabel->pixmap()->scaled(newDims.first,newDims.second,
              Qt::IgnoreAspectRatio,
              Qt::SmoothTransformation));
      initialMatchingLabel->resize(newDims.first, newDims.second);
    }
 
    if(_finalMatched)
    {
      newDims = checkDimensions(finalMatchingLabel->pixmap()->width(), finalMatchingLabel->pixmap()->height(),
                             this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

      finalMatchingLabel->setPixmap(finalMatchingLabel->pixmap()->scaled(newDims.first,newDims.second,
              Qt::IgnoreAspectRatio,
              Qt::SmoothTransformation));
      finalMatchingLabel->resize(newDims.first, newDims.second);
    }

    if(_merged)
    {
      newDims = checkDimensions(mergedLabel->pixmap()->width(), mergedLabel->pixmap()->height(),
                             this->stackedWidget->width(), this->stackedWidget->height() - 20, false);

      mergedLabel->setPixmap(mergedLabel->pixmap()->scaled(newDims.first,newDims.second,
              Qt::IgnoreAspectRatio,
              Qt::SmoothTransformation));
      mergedLabel->resize(newDims.first, newDims.second);
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
  @brief Draws voronoi diagram in the map
  @param img [QImage*] The image for the diagram to be drawn on
  @param voronoi [std::vector<geometry_msgs::Point>*] The voronoi diagram
  @return void
  **/
  void CMapLoader::drawVoronoi(QImage *img, std::vector<geometry_msgs::Point>* voronoi, bool groundTruth)
  {
    QPainter painter(img);
    if (groundTruth)
      painter.setPen(QColor(255, 0, 0));
    else
      painter.setPen(QColor(0, 0, 255));
    for (int i = 0; i < voronoi->size(); i++)
      painter.drawPoint(voronoi->at(i).y, voronoi->at(i).x);
  }
 
  /**
  @brief Draws voronoi vertices in the map
  @param img [QImage*] The image for the vertices to be drawn on
  @param voronoi [std::vector<geometry_msgs::Point>*] The voronoi vertices
  @return void
  **/
  void CMapLoader::drawVertices(QImage *img, std::vector<geometry_msgs::Point>* vertices, bool groundTruth)
  {
    QPainter painter(img);
    QPen pen;
    if(groundTruth)
       pen = QPen((QColor(0, 255, 0)));
    else
       pen = QPen((QColor(0, 255, 255)));
    pen.setWidth(5);
    painter.setPen(pen);
    for (int i = 0; i < vertices->size(); i++)
      painter.drawPoint(vertices->at(i).y, vertices->at(i).x);

  }

  /**
  @brief Returns the pos of the object to the scene
  @return QPointF : The pos to the scene
  **/
  QPointF CMapLoader::getPosition()
  {
    return ground_truth_map->pos();
  }

  /**
  @brief Returns the rotation (z-axis) of the object to the scene
  @return qreal : The rotation to the scene
  **/
  qreal CMapLoader::getRotation()
  {
    return ground_truth_map->rotation();
  }

  /**
  @brief Returns the scaling of the object
  @return qreal : The scaling
  **/
  qreal CMapLoader::getScale()
  {
    return ground_truth_map->scale();
  }

  /**
  @brief Returns the scaling of the slam Map
  @return qreal : The scaling
  **/
  qreal CMapLoader::getSlamMapScale()
  {
    ROS_INFO_STREAM("SLAMoffsetScale=" << slamOffsetScale);
    return slamOffsetScale;
  }

  /**
  @brief Returns the scaling of the slam Map
  @return qreal : The scaling
  **/
  qreal CMapLoader::getGroundTruthMapScale()
  {

    ROS_INFO_STREAM("GroundoffsetScale=" << groundTruthOffsetScale);
    return groundTruthOffsetScale;
  }

  /**
  @brief Returns the item's scene transformation matrix
  @return QTransform : The transformation matrix
  **/
  QTransform CMapLoader::getTransform()
  {
    return ground_truth_map->sceneTransform();
  }

}
