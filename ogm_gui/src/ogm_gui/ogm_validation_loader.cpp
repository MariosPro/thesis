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
  
    obstaclePushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_down.png")).c_str())));
 
    obstacleSettingsPushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/properties.png")).c_str())));

    featuresPushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_down.png")).c_str())));
 
    featuresSettingsPushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/properties.png")).c_str())));

    posxSpinBox->setRange(-1000, 1000);
    posySpinBox->setRange(-1000, 1000);
    rotationSpinBox->setRange(0, 360);
    rotationSpinBox->setWrapping(true);
    transSpinBox->setRange(0.00, 1.00);
    transSpinBox->setSingleStep(0.1);
    scaleSpinBox->setSingleStep(0.05);

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
/*    int count = mapInfo.childCount();*/
    //for(int i = count - 1 ; i >= 0 ; i--)
    //{
      ////~ deleteTreeNode(mapInfo.child(i));
      //QTreeWidgetItem *child = mapInfo.child(i);
      //mapInfo.removeChild(mapInfo.child(i));
      ////~ delete child;
    /*}*/
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
     if (groundTruth)
      {
        groundMapHeightLabel->setText("Ground Truth Map Height: " + QString().setNum(height) + QString(" m"));
        groundMapWidthLabel->setText("Ground Truth Map Width: " + QString().setNum(width) + QString(" m"));
        groundMapResLabel->setText("Ground Truth Map Resolution: " + QString().setNum(ocgd) + QString(" m/px"));
      }
      else
      {
        producedMapHeightLabel->setText("SLAM Produced Map Height: " + QString().setNum(height) + QString(" m"));
        producedMapWidthLabel->setText("SLAM Produced Map Width: " + QString().setNum(width) + QString(" m"));
        producedMapResLabel->setText("SLAM Produced Map Resolution: " + QString().setNum(ocgd) + QString(" m/px"));
      }
  }

  void CValidationLoader::showMapXposition(qreal x)
  {
       posxLabel->setText("Translation-x : " + QString().setNum(x) );
  }
  
  void CValidationLoader::showMapYposition(qreal y)
  {
    posyLabel->setText("Translation-y : " + QString().setNum(y) );
  }
 
  void CValidationLoader::showMapRotation(qreal r)
  {
    rotationLabel->setText("Rotation-z axis : "+ QString().setNum(r));
  }

  void CValidationLoader::showMapScale(qreal s)
  {
    scaleLabel->setText("Scaling Factor : "+ QString().setNum(s));
  }

  void CValidationLoader::showMapTransparency(double t)
  {
    transLabel->setText("Transparency : "+ QString().setNum(t));
  }

  void CValidationLoader::resetMapProperties()
  {
    showMapXposition(0);
    showMapYposition(0);
    showMapRotation(0);
    showMapScale(1.0);
    showMapTransparency(0.5);
    posxSpinBox->setValue(0);
    posySpinBox->setValue(0);
    rotationSpinBox->setValue(0);
    scaleSpinBox->setValue(1);
    transSpinBox->setValue(0.5);
  }


}

