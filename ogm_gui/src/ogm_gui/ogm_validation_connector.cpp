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

#include "ogm_gui/ogm_validation_connector.h"

namespace ogm_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CValidationConnector::CValidationConnector(int argc, char **argv):
    QObject(),
    loader(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    /*QObject::connect(*/
      //loader.ogmInformationTree,
        //SIGNAL(itemClicked(QTreeWidgetItem*, int)),
      //this,
        //SLOT(treeItemClicked(QTreeWidgetItem*, int)));
        
    //QObject::connect(
      //this,
        //SIGNAL(adaptSignal()),
      //this,
        //SLOT(adaptSlot()));
        
    //QObject::connect(
      //loader.ogmInformationTree, 
        //SIGNAL(itemCollapsed(QTreeWidgetItem *)),
      //this, 
        //SLOT(adaptColumns(QTreeWidgetItem *)));
        
    //QObject::connect(
      //loader.ogmInformationTree, 
        //SIGNAL(itemExpanded(QTreeWidgetItem *)),
      //this, 
        /*SLOT(adaptColumns(QTreeWidgetItem *)));*/

    QObject::connect(
      loader.posxSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(posxChanged(double)));

    QObject::connect(
      loader.posySpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(posyChanged(double)));

    QObject::connect(
      loader.rotationSpinBox, SIGNAL(valueChanged(int)),
      this, SLOT(rotationChanged(int)));

   QObject::connect(
      loader.scaleSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(scaleChanged(double)));
 
   QObject::connect(
      loader.transSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(transparencyChanged(double)));

   QObject::connect(
      loader.obstaclePushButton, SIGNAL(clicked(bool)),
      this, SLOT(obstacleTriggered()));
 
   QObject::connect(
      loader.obstacleSettingsPushButton, SIGNAL(clicked(bool)),
      this, SLOT(obstacleSettingsTriggered()));

   QObject::connect(
      loader.featuresPushButton, SIGNAL(clicked(bool)),
      this, SLOT(featureMatchingTriggered()));

   QObject::connect(
      loader.featuresSettingsPushButton, SIGNAL(clicked(bool)),
      this, SLOT(featureMatchingSettingsTriggered()));

  }

  /**
  @brief Adapts the columns width according to what is visible when an item is clicked
  @param item [QTreeWidgetItem*] Item clicked
  @param column [int] Column clicked
  @return void
  **/
  void CValidationConnector::adaptColumns(QTreeWidgetItem *item, int column)
  {
/*    loader.ogmInformationTree->resizeColumnToContents(0);*/
    //loader.ogmInformationTree->resizeColumnToContents(1);
    //loader.ogmInformationTree->resizeColumnToContents(2);
    /*loader.ogmInformationTree->resizeColumnToContents(3);*/
  }
  
  /**
  @brief Adapts the columns width according to what is visible when an item expands or collapses
  @param item [QTreeWidgetItem*] Item expanded / collapsed
  @return void
  **/
  void CValidationConnector::adaptColumns(QTreeWidgetItem *item)
  {
/*    loader.ogmInformationTree->resizeColumnToContents(0);*/
    //loader.ogmInformationTree->resizeColumnToContents(1);
    //loader.ogmInformationTree->resizeColumnToContents(2);
    /*loader.ogmInformationTree->resizeColumnToContents(3);*/
  }
  
  /**
  @brief Adapts the columns width according to what is visible. Called when adaptSignal is emmited
  @return void
  **/
  void CValidationConnector::adaptSlot(void)
  {
/*    loader.ogmInformationTree->resizeColumnToContents(0);*/
    //loader.ogmInformationTree->resizeColumnToContents(1);
    //loader.ogmInformationTree->resizeColumnToContents(2);
    /*loader.ogmInformationTree->resizeColumnToContents(3);*/
  }

  /**
  @brief Updates the information tree according to the specific map
  @param width [float] The map width
  @param height [float] The map height
  @param ocgd [float] The map resolution (m/pixel)
  @return void
  **/
  void CValidationConnector::updateMapInfo(const ogm_msgs::MapsMsg& msg)
  {
    //loader.deleteTree();
    loader.updateMapInfo(msg.groundTruthMap.info.width * msg.groundTruthMap.info.resolution,
                         msg.groundTruthMap.info.height * msg.groundTruthMap.info.resolution,
                         msg.groundTruthMap.info.resolution, true);
    loader.updateMapInfo(msg.slamMap.info.width * msg.slamMap.info.resolution,
                         msg.slamMap.info.height * msg.slamMap.info.resolution,
                         msg.slamMap.info.resolution, false);
    //Q_EMIT adaptSignal();
  }

  /**
  @brief Called when a click occurs in the tree
  @param item [QTreeWidgetItem*] Item clicked
  @param column [int] Column clicked
  @return void
  **/
  void CValidationConnector::treeItemClicked(QTreeWidgetItem * item, int column)
  {

  }

  void CValidationConnector::posxChanged(double x)
  {
    Q_EMIT changeXPos(x);
    loader.showMapXposition(x);
  }

  void CValidationConnector::posyChanged(double y)
  {
    Q_EMIT changeYPos(y);
    loader.showMapYposition(y);
  }

  void CValidationConnector::rotationChanged(int r)
  {
    Q_EMIT changeRotation(r);
    loader.showMapRotation(r);
  }

  void CValidationConnector::scaleChanged(double s)
  {
    Q_EMIT changeScale(s);
    loader.showMapScale(s);
  }

  void CValidationConnector::transparencyChanged(double t)
  {
    Q_EMIT changeTransparency(t);
    loader.showMapTransparency(t);
  }

  void CValidationConnector::showMapPosition(qreal x, qreal y)
  {
    loader.showMapXposition(x);
    loader.showMapYposition(y);
  }

  void CValidationConnector::showMapRotation(qreal r)
  {
    loader.showMapRotation(r);
  }

  void CValidationConnector::showMapScale(qreal s)
  {
    loader.showMapScale(s);
  }

  void CValidationConnector::showMapTransparency(double t)
  {
    loader.showMapTransparency(t);
  }

  void CValidationConnector::resetMapProperties()
  {
    loader.resetMapProperties();
  }

  void CValidationConnector::obstacleTriggered()
  {
    Q_EMIT MetricNeeded("OMSE");
  }

  void CValidationConnector::obstacleSettingsTriggered()
  {

  }

  void CValidationConnector::featureMatchingTriggered()
  {
    Q_EMIT MetricNeeded("FEATURES");
    //TO DO call featureMatchingSettingsTriggered (default values)
  }

  void CValidationConnector::featureMatchingSettingsTriggered()
  {

  }

  void CValidationConnector::displayMetricResult(QString method, double result)
  {
    if (method == "OMSE")
      loader.obstacleLabel->setText(QString().setNum(result));
    if (method == "FEATURES")
      loader.featuresLabel->setText(QString().setNum(result));
  }

  /**
  @brief Returns the CValidationLoader object
  @return QWidget*
  **/
  QWidget* CValidationConnector::getLoader(void)
  {
    return static_cast<QWidget *>(&loader);
  }
}

