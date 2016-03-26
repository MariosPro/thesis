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
      loader.matchingRatioSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(matchingRatioChanged(double)));

   QObject::connect(
      loader.ransacSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(ransacChanged(double)));

   QObject::connect(
      loader.obstaclePushButton, SIGNAL(clicked(bool)),
      this, SLOT(obstacleTriggered()));

   QObject::connect(
      loader.detectorComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(detectorComboBoxActivated(const QString&)));

   QObject::connect(
      loader.descriptorComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(descriptorComboBoxActivated(const QString&)));

   QObject::connect(
      loader.matcherComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(matcherComboBoxActivated(const QString&)));
  
   QObject::connect(
      loader.matchingMethodComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(matchingMethodComboBoxActivated(const QString&)));

   QObject::connect(
      loader.closestObstacleComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(closestObstacleComboBoxActivated(const QString&)));

   QObject::connect(
      loader.distanceComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(distanceComboBoxActivated(const QString&)));

   QObject::connect(
      loader.featuresPushButton, SIGNAL(clicked(bool)),
      this, SLOT(featureMatchingTriggered()));
 
   QObject::connect(
      loader.binaryCheckBox, SIGNAL(toggled(bool)),
      this, SLOT(binaryMapsChecked()));
 
   QObject::connect(
      loader.manualAlignmentCheckBox, SIGNAL(toggled(bool)),
      this, SLOT(manualAlignmentChecked()));


   _detector = "SIFT";
   _descriptor = "SIFT";
   _matcher = "BruteForce";
   _matchingMethod = "SIMPLE";
   _closestObstacleMethod = "Brushfire";
   _distMethod = "Manhattan";
   _matchingRatio = 0.7;
   _ransacReprjError = 5;
   _binary = false;
   _manual_alignment = false;

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
    loader.updateMapInfo(msg.groundTruthMap.info.width * msg.groundTruthMap.info.resolution,
                         msg.groundTruthMap.info.height * msg.groundTruthMap.info.resolution,
                         msg.groundTruthMap.info.resolution, true);
    loader.updateMapInfo(msg.slamMap.info.width * msg.slamMap.info.resolution,
                         msg.slamMap.info.height * msg.slamMap.info.resolution,
                         msg.slamMap.info.resolution, false);
  }

  std::string CValidationConnector::getDetector()
  {
    return _detector;
  }

  std::string CValidationConnector::getDescriptor()
  {
    return _descriptor;
  }

  std::string CValidationConnector::getMatcher()
  {
    return _matcher;
  }
 
  std::string CValidationConnector::getMatchingMethod()
  {
    return _matchingMethod;
  }


  std::string CValidationConnector::getClosestObstacleMethod()
  {
    return _closestObstacleMethod;
  }

  std::string CValidationConnector::getDistanceMethod()
  {
    return _distMethod;
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
 
  void CValidationConnector::matchingRatioChanged(double t)
  {
    _matchingRatio = t;
  }

  void CValidationConnector::ransacChanged(double t)
  {
    _ransacReprjError = t;
  }

  double CValidationConnector::getMatchingRatio()
  {
    return _matchingRatio;
  }
  
  double CValidationConnector::getRansacReprjError()
  {
    return _ransacReprjError;
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

  void CValidationConnector::featureMatchingTriggered()
  {
    Q_EMIT MetricNeeded("FEATURES");
    //TO DO call featureMatchingSettingsTriggered (default values)
  }

  void CValidationConnector::detectorComboBoxActivated(const QString& text)
  {
    _detector = text.toStdString();
    ROS_INFO_STREAM("DETECTOR:" << text.toStdString());
  }
 
  void CValidationConnector::descriptorComboBoxActivated(const QString& text)
  {
    _descriptor = text.toStdString();
    ROS_INFO_STREAM("DESCRIPTOR:" << text.toStdString());
  }

  void CValidationConnector::matcherComboBoxActivated(const QString& text)
  {
    _matcher = text.toStdString();
    ROS_INFO_STREAM("MATCHER:" << text.toStdString());
  }
 
  void CValidationConnector::matchingMethodComboBoxActivated(const QString& text)
  {
    _matchingMethod = text.toStdString();
    ROS_INFO_STREAM("MATCHING METHOD:" << text.toStdString());
  }

  void CValidationConnector::closestObstacleComboBoxActivated(const QString& text)
  {
    _closestObstacleMethod = text.toStdString();
    ROS_INFO_STREAM("CLOSEST OBSTACLE METHOD: " << text.toStdString());
  }

  void CValidationConnector::distanceComboBoxActivated(const QString& text)
  {
    _distMethod = text.toStdString();
    ROS_INFO_STREAM("DISTANCE METHOD: " << text.toStdString());
  }

  void CValidationConnector::displayMetricResult(QString method, double result)
  {
    if (method == "OMSE")
      loader.obstacleLabel->setText(QString().setNum(result));
    if (method == "FEATURES")
      loader.featuresLabel->setText(QString().setNum(result));
  }

  void CValidationConnector::binaryMapsChecked()
  {
    _binary = !_binary;
  }

  void CValidationConnector::manualAlignmentChecked()
  {
    _manual_alignment = !_manual_alignment;
  }


  bool CValidationConnector::thresholdMaps()
  {
    return _binary;
  }
 
  bool CValidationConnector::manualAlignMaps()
  {
    return _manual_alignment;
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

