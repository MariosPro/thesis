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
      loader.gaussianBlurSpinBox1, SIGNAL(valueChanged(double)),
      this, SLOT(gaussianBlurSpinBox1Changed(double)));
  
   QObject::connect(
      loader.gaussianBlurSpinBox2, SIGNAL(valueChanged(double)),
      this, SLOT(gaussianBlurSpinBox2Changed(double)));
  
   QObject::connect(
      loader.medianBlurSpinBox1, SIGNAL(valueChanged(double)),
      this, SLOT(medianBlurSpinBox1Changed(double)));
  
   QObject::connect(
      loader.medianBlurSpinBox2, SIGNAL(valueChanged(double)),
      this, SLOT(medianBlurSpinBox2Changed(double)));

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

   QObject::connect(
      loader.scaleBrushfireCheckBox, SIGNAL(toggled(bool)),
      this, SLOT(scaleMapsBrushfireChecked()));
 
   QObject::connect(
      loader.gaussianBlurCheckBox1, SIGNAL(toggled(bool)),
      this, SLOT(gaussianBlurCheckBox1Checked()));
  
   QObject::connect(
      loader.gaussianBlurCheckBox2, SIGNAL(toggled(bool)),
      this, SLOT(gaussianBlurCheckBox2Checked()));
  
   QObject::connect(
      loader.medianBlurCheckBox1, SIGNAL(toggled(bool)),
      this, SLOT(medianBlurCheckBox1Checked()));
  
   QObject::connect(
      loader.medianBlurCheckBox2, SIGNAL(toggled(bool)),
      this, SLOT(medianBlurCheckBox2Checked()));
  
   // structural evaluation
   QObject::connect(
      loader.graphMatchingPushButton, SIGNAL(clicked(bool)),
      this, SIGNAL(requestStructuralEvaluation()));
 
   QObject::connect(
      loader.gaussianBlurCheckBox1_2, SIGNAL(toggled(bool)),
      this, SLOT(gaussianBlurCheckBox1Checked()));
  
   QObject::connect(
      loader.gaussianBlurCheckBox2_2, SIGNAL(toggled(bool)),
      this, SLOT(gaussianBlurCheckBox2Checked()));
  
   QObject::connect(
      loader.medianBlurCheckBox1_2, SIGNAL(toggled(bool)),
      this, SLOT(medianBlurCheckBox1Checked()));
  
   QObject::connect(
      loader.medianBlurCheckBox2_2, SIGNAL(toggled(bool)),
      this, SLOT(medianBlurCheckBox2Checked()));
  
   QObject::connect(
      loader.morphOpeningCheckBox, SIGNAL(toggled(bool)),
      this, SLOT(morphOpeningCheckBoxChecked()));
 
   QObject::connect(
      loader.pruningCheckBox, SIGNAL(toggled(bool)),
      this, SLOT(pruningCheckBoxChecked()));

   QObject::connect(
      loader.skeletonizationComboBox, SIGNAL(currentIndexChanged(const QString&)),
      this, SLOT(skeletonizationComboBoxActivated(const QString&)));
 
   QObject::connect(
      loader.gaussianBlurSpinBox1_2, SIGNAL(valueChanged(double)),
      this, SLOT(gaussianBlurSpinBox1Changed(double)));
  
   QObject::connect(
      loader.gaussianBlurSpinBox2_2, SIGNAL(valueChanged(double)),
      this, SLOT(gaussianBlurSpinBox2Changed(double)));
  
   QObject::connect(
      loader.medianBlurSpinBox1_2, SIGNAL(valueChanged(double)),
      this, SLOT(medianBlurSpinBox1Changed(double)));
  
   QObject::connect(
      loader.medianBlurSpinBox2_2, SIGNAL(valueChanged(double)),
      this, SLOT(medianBlurSpinBox2Changed(double)));
 
   QObject::connect(
      loader.pruningSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(pruningSpinBoxChanged(double)));
 
   QObject::connect(
      loader.morphOpeningSpinBox, SIGNAL(valueChanged(double)),
      this, SLOT(morphOpeningSpinBoxChanged(double)));


  // feature evaluation
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
  _scaleMapsBrushfire = false;
  _gaussianBlur1 = false;
  _gaussianBlur2 = false;
  _medianBlur1 = false;
  _medianBlur2 = false;
  _gaussianKernel1 = 3;
  _gaussianKernel2 = 3;
  _medianKernel1 = 3;
  _medianKernel2 = 3;

  //structural evaluation
  _morphOpenIterations = 4;
  _pruningIterations = 20;
  _skeletonizationMethod = "medial_axis";
  _pruning = false;
  _morphOpen = false;
    
  }

   /**
  @brief Updates the information tree according to the specific map
  @param width [float] The map width
  @param height [float] The map height
  @param ocgd [float] The map resolution (m/pixel)
  @return void
  **/
  void CValidationConnector::updateMapInfo(const ogm_communications::MapsMsg& msg)
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

  void CValidationConnector::gaussianBlurSpinBox1Changed(double t)
  {
    _gaussianKernel1 = t;
  }

  void CValidationConnector::gaussianBlurSpinBox2Changed(double t)
  {
    _gaussianKernel2 = t;
  }
 
  void CValidationConnector::medianBlurSpinBox1Changed(double t)
  {
    _medianKernel1 = t;
  }

  void CValidationConnector::medianBlurSpinBox2Changed(double t)
  {
    _medianKernel2 = t;
  }


  double CValidationConnector::getMatchingRatio()
  {
    return _matchingRatio;
  }
  
  double CValidationConnector::getRansacReprjError()
  {
    return _ransacReprjError;
  }

  double CValidationConnector:: getGaussianKernel1()
  {
    return _gaussianKernel1;
  }
  
  double CValidationConnector:: getGaussianKernel2()
  {
    return _gaussianKernel2;
  }
  
  double CValidationConnector:: getMedianKernel1()
  {
    return _medianKernel1;
  }
 
  double CValidationConnector:: getMedianKernel2()
  {
    return _medianKernel2;
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

  void CValidationConnector::scaleMapsBrushfireChecked()
  {
    _scaleMapsBrushfire = !_scaleMapsBrushfire;
  }
 
  void CValidationConnector::gaussianBlurCheckBox1Checked()
  {
    _gaussianBlur1 = !_gaussianBlur1;
  }
 
  void CValidationConnector::gaussianBlurCheckBox2Checked()
  {
    _gaussianBlur2 = !_gaussianBlur2;
  }
 
  void CValidationConnector::medianBlurCheckBox1Checked()
  {
    _medianBlur1 = !_medianBlur1;
  }


  void CValidationConnector::medianBlurCheckBox2Checked()
  {
    _medianBlur2 = !_medianBlur2;
  }



  bool CValidationConnector::thresholdMaps()
  {
    return _binary;
  }
 
  bool CValidationConnector::manualAlignMaps()
  {
    return _manual_alignment;
  }

  bool CValidationConnector::scaleMapsBrushfire()
  {
    return _scaleMapsBrushfire;
  }

  bool CValidationConnector::gaussianBlur1()
  {
    return _gaussianBlur1;
  }
 
  bool CValidationConnector::medianBlur1()
  {
    return _medianBlur1;
  }

  bool CValidationConnector::gaussianBlur2()
  {
    return _gaussianBlur2;
  }

  bool CValidationConnector::medianBlur2()
  {
    return _medianBlur2;
  }

  // structrural evaluation slots
  void CValidationConnector::morphOpeningCheckBoxChecked()
  {
    _morphOpen = !_morphOpen;
  }

  void CValidationConnector::pruningCheckBoxChecked()
  {
    _pruning = !_pruning;
  }

  void CValidationConnector::skeletonizationComboBoxActivated(const QString& text)
  {
    _skeletonizationMethod = text.toStdString();
    ROS_INFO_STREAM("SKELETONIZATION METHOD: " << text.toStdString());
  }

  void CValidationConnector::morphOpeningSpinBoxChanged(double t)
  {
    _morphOpenIterations = t;
  }

  void CValidationConnector::pruningSpinBoxChanged(double t)
  {
    _pruningIterations = t;
  }

  //structural evaluation getters
  bool CValidationConnector::pruning()
  {
    return _pruning;
  }
  
  bool CValidationConnector::morphOpen()
  {
    return _morphOpen;
  }

  double CValidationConnector::getMorphOpenIterations()
  {
    return _morphOpenIterations;
  }

  double CValidationConnector::getPruningIterations()
  {
    return _pruningIterations;
  }

  std::string CValidationConnector::getSkeletonizationMethod()
  {
    return _skeletonizationMethod;
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

