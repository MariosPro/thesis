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
#include "ogm_msgs/MapsMsg.h"

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
      void updateMapInfo(const ogm_msgs::MapsMsg& msg);

      /**
      @brief Returns the CValidationLoader object
      @return QWidget*
      **/
      QWidget* getLoader(void);


    //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief Called when a click occurs in the tree
      @param item [QTreeWidgetItem*] Item clicked
      @param column [int] Column clicked
      @return void
      **/
      void treeItemClicked(QTreeWidgetItem * item, int column ); 

      /**
      @brief Adapts the columns width according to what is visible when an item is clicked
      @param item [QTreeWidgetItem*] Item clicked
      @param column [int] Column clicked
      @return void
      **/
      void adaptColumns(QTreeWidgetItem *item, int column);

      /**
      @brief Adapts the columns width according to what is visible when an item expands or collapses
      @param item [QTreeWidgetItem*] Item expanded / collapsed
      @return void
      **/
      void adaptColumns(QTreeWidgetItem *item);

      /**
      @brief Adapts the columns width according to what is visible. Called when adaptSignal is emmited
      @return void
      **/
      void adaptSlot(void);

      void posxChanged(int x);

      void posyChanged(int y);
      
      void rotationChanged(int r);

      void scaleChanged(double s);
      
      void transparencyChanged(double t);

    //------------------------------------------------------------------------//
    Q_SIGNALS:

      /**
      @brief Emmited when resize of columns according to whats visible is needed
      @return void
      **/
      void adaptSignal(void);

      void changeXPos(int x);
 
      void changeYPos(int y);

      void changeRotation(int r);

      void changeScale(double s);

      void changeTransparency(double t);

  };
}
#endif



