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

#include "ogm_gui/ogm_validation_loader.h"

namespace ogm_gui
{

  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CValidationLoader::CValidationLoader(int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    int x = this->validationTabWidget->indexOf(metricsTab);
    ROS_INFO_STREAM("X=" <<  x);
    ogmInformationTree->header()->setDefaultSectionSize(20);
    ogmInformationTree->header()->setMinimumSectionSize(10);
    ogmInformationTree->setColumnCount(4);
    ogmInformationTree->setColumnWidth(0, 20);
    ogmInformationTree->setColumnWidth(1, 20);
    ogmInformationTree->setColumnWidth(2, 20);
    ogmInformationTree->setColumnWidth(3, 20);
    
    QStringList ColumnNames;
    ColumnNames << "" << "" << "" << "" << "";
 
    ogmInformationTree->setHeaderLabels(ColumnNames);
    
    mapInfo.setText(0,"Maps Information");
    mapEvaluation.setText(0, "Map Evaluation");
    pathEvaluation.setText(0, "Path Evaluation");
    metrics.setText(0, "Image Evaluation General Metrics");
    structEvaluation.setText(0, "Structural Evaluation");
    featEvaluation.setText(0, "Feature-based Evaluation");
    mapEvaluation.addChild(&metrics);
    mapEvaluation.addChild(&structEvaluation);
    mapEvaluation.addChild(&featEvaluation);
     
    ogmInformationTree->addTopLevelItem(&mapInfo);
    ogmInformationTree->addTopLevelItem(&mapEvaluation);
    ogmInformationTree->addTopLevelItem(&pathEvaluation);
    mapInfo.setExpanded(true);
    mapEvaluation.setExpanded(true);
    pathEvaluation.setExpanded(true);

  
    rotationSpinBox->setRange(-360, 360);
    transSpinBox->setRange(0.00, 1.00);
    transSpinBox->setSingleStep(0.1);

  }

  /**
  @brief Default destructor
  @return void
  +**/
  CValidationLoader::~CValidationLoader(void)
  {

  }

  /**
  @brief Deletes a specific tree node. Recursive function.
  @param item [QTreeWidgetItem*] The item to be deleted
  @return void
  **/
  void CValidationLoader::deleteTreeNode(QTreeWidgetItem *item)
  {
    int count = item->childCount();

    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(item->child(i));
      QTreeWidgetItem *child = item->child(i);
      item->removeChild(item->child(i));
      delete child;
    }
  }

  /**
  @brief Deletes the information tree
  @return void
  **/
  void CValidationLoader::deleteTree(void)
  {
    int count = mapInfo.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      //~ deleteTreeNode(mapInfo.child(i));
      QTreeWidgetItem *child = mapInfo.child(i);
      mapInfo.removeChild(mapInfo.child(i));
      //~ delete child;
    }
  }

  /**
  @brief Updates the information tree according to the specific map
  @param width [float] The map width
  @param height [float] The map height
  @param ocgd [float] The map resolution (m/pixel)
  @return void
  **/
  void CValidationLoader::updateMapInfo(float width, float height, float ocgd, bool groundTruth)
  {
      QTreeWidgetItem *mnode = new QTreeWidgetItem();
      if(groundTruth)
       mnode->setText(0, QString("Ground Truth Map"));
      else
       mnode->setText(0, QString("Slam-Produced Map"));

      QTreeWidgetItem *mapWidth = new QTreeWidgetItem(),
                    *mapHeight = new QTreeWidgetItem(),
                    *mapOcgd = new QTreeWidgetItem();

      mapWidth->setText(0,"Map Width");
      mapWidth->setText(1, (QString().setNum(width) + QString(" m")));
      mapHeight->setText(0, "Map Height");
      mapHeight->setText(1, (QString().setNum(height) + QString(" m")));
      mapOcgd->setText(0,"Map Resolution");
      mapOcgd->setText(1, (QString().setNum(ocgd) + QString(" m/px")));
      mnode->addChild(mapWidth);
      mnode->addChild(mapHeight);
      mnode->addChild(mapOcgd);
      mapInfo.addChild(mnode);
  }
}

