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

#ifndef OGM_VALIDATION_CONNECTOR
#define OGM_VALIDATION_CONNECTOR

#include "ogm_gui/ogm_validation_loader.h"
#include "ogm_communications/MapsMsg.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class CValidationConnector
  @brief Serves the Qt events of the OGM validation Tabwidget. Inherits from QObject
  **/
  class CValidationConnector :
    public QObject
  {
    Q_OBJECT
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;

      //!< the Feature Detector to be implemented
      std::string _detector;

      //!< the Feature Descriptor to be implemented
      std::string _descriptor;
 
      //!< the Feature matcher to be implemented
      std::string _matcher;
   
      //!< the Feature matching Method to be used
      std::string _matchingMethod;

      //!< the closestObstacle method to be implemented
      std::string _closestObstacleMethod;
  
      //!< the distance  method to be used
      std::string _distMethod;

      double _matchingRatio;

      double _ransacReprjError;

      double _medianKernel1;

      double _medianKernel2;

      double _gaussianKernel1;

      double _gaussianKernel2;

      bool _binary;

      bool _manual_alignment;

      bool _scaleMapsBrushfire;

      bool _gaussianBlur1;

      bool _gaussianBlur2;

      bool _medianBlur1;
      
      bool _medianBlur2;

      //structural Evaluation Parameters
      
      bool _morphOpen;

      bool _pruning;

      bool _voronoi;

      bool _vertices;

      bool _graphMatching;

      std::string _skeletonizationMethod;

      double _pruningIterations;

      double _morphOpenIterations;

    //------------------------------------------------------------------------//
    public:

      //!< Object of CValidationLoader
      CValidationLoader loader;

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CValidationConnector(int argc, char **argv);

      /**
      @brief Updates the information tree according to the specific map
      @param width [float] The map width
      @param height [float] The map height
      @param ocgd [float] The map resolution (m/pixel)
      @return void
      **/
      void updateMapInfo(const ogm_communications::MapsMsg& msg);

      void resetMapProperties();

      void displayMetricResult(QString method, double result);

      /**
      @brief Returns the CValidationLoader object
      @return QWidget*
      **/
      QWidget* getLoader(void);

      std::string getDescriptor();

      std::string getDetector();

      std::string getMatcher();
  
      std::string getMatchingMethod();
 
      std::string getClosestObstacleMethod();

      std::string getDistanceMethod();

      double getMatchingRatio();

      double getRansacReprjError();

      double getGaussianKernel1();

      double getGaussianKernel2();

      double getMedianKernel1();

      double getMedianKernel2();

      bool thresholdMaps();

      bool manualAlignMaps();

      bool scaleMapsBrushfire();

      bool gaussianBlur1();

      bool gaussianBlur2();

      bool medianBlur1();

      bool medianBlur2();

      //structural evaluation getters
      std::string getSkeletonizationMethod();

      double getPruningIterations();

      double getMorphOpenIterations();

      bool pruning();

      bool morphOpen();

      bool isVoronoiEnabled();

      bool isVerticesEnabled();

      bool isGraphMatchingEnabled();
    //------------------------------------------------------------------------//
    public Q_SLOTS:

      void posxChanged(double x);

      void posyChanged(double y);
      
      void rotationChanged(int r);

      void scaleChanged(double s);
      
      void transparencyChanged(double t);

      void matchingRatioChanged(double t);

      void ransacChanged(double t);

      void gaussianBlurSpinBox1Changed(double t);

      void gaussianBlurSpinBox2Changed(double t);

      void medianBlurSpinBox1Changed(double t);

      void medianBlurSpinBox2Changed(double t);

      void showMapPosition(qreal x, qreal y);

      void showMapRotation(qreal r);

      void showMapScale(qreal s);

      void showMapTransparency(double t);

      void obstacleTriggered();

      void featureMatchingTriggered();

      void detectorComboBoxActivated(const QString& text);

      void descriptorComboBoxActivated(const QString& text);

      void matcherComboBoxActivated(const QString& text);

      void matchingMethodComboBoxActivated(const QString& text);

      void closestObstacleComboBoxActivated(const QString& text);

      void distanceComboBoxActivated(const QString& text);

      void binaryMapsChecked();

      void manualAlignmentChecked();

      void scaleMapsBrushfireChecked();

      void gaussianBlurCheckBox1Checked();

      void gaussianBlurCheckBox2Checked();

      void medianBlurCheckBox1Checked();

      void medianBlurCheckBox2Checked();

      // structural evaluation
      void morphOpeningCheckBoxChecked();

      void pruningCheckBoxChecked();

      void voronoiCheckBoxChecked();

      void verticesCheckBoxChecked();

      void graphMatchingCheckBoxChecked();

      void pruningSpinBoxChanged(double t);

      void morphOpeningSpinBoxChanged(double t);

      void skeletonizationComboBoxActivated(const QString& text);

    //------------------------------------------------------------------------//
    Q_SIGNALS:

      void changeXPos(double x);
 
      void changeYPos(double y);

      void changeRotation(int r);

      void changeScale(double s);

      void changeTransparency(double t);

      void MetricNeeded(QString metricMethod);

      void requestStructuralEvaluation();
  };
}
#endif



