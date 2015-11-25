/******************************************************************************
   OGM Validator -Occupancy Grid Map Validator
   Copyright (C) 2015 OGM  Validator
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

#ifndef OGM_GUI_CONNECTOR
#define OGM_GUI_CONNECTOR

#include "ogm_server/map_loader.h"
#include <ogm_msgs/LoadExternalMap.h>
#include <ogm_msgs/MapMsg.h>
#include "ogm_gui/ogm_gui_loader.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{

  /**
  @class CGuiConnector
  @brief Serves the Qt events of the main GUI window. Inherits from QObject
  **/
  class CGuiConnector:
    public QObject
  {
    Q_OBJECT

    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;
      //!< True if any map is loaded from server
      bool map_initialized_;
      //!< True if grid is enabled
      bool   grid_enabled_;

      //!< The loader of main GUI QWidget
      CGuiLoader loader_;

    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CGuiConnector(int argc, char **argv);

      /**
      @brief Returns the grid enabled state
      @return bool : True if grid is enabled
      **/
      bool isGridEnabled(void);

       /** 
       @brief Raises a message box with a specific message
       @param title [QString] The message box title
       @param s [QString] The message
       @return void
       **/
       void raiseMessage(QString title, QString s); 

      /**
      @brief Adds a widget to the main window Qt grid
      @param w [QWidget*] The widget to be placed
      @param row [int] The row of the grid
      @param column [int] The column of the grid
      @return void
      **/
      void addToGrid(QWidget *w, int row, int column, 
                     int rowSpan, int columnSpan);

      /**
      @brief Wraps the Qt gridColumnStretch function
      @param cell [int] The specific column
      @param stretch [int] The relative stretch coefficient
      @return void
      **/
      void setGridColumnStretch(int cell,int stretch);

      /**
      @brief Shows the main window
      @return void
      **/
      void show(void);

      /**
      @brief Displays a message in the QMainWindow status bar
      @param s [QString] The message
      @return void
      **/
      void setStatusBarMessage(QString s);

      /**
      @brief Returns the exit event captured
      @return QEvent* The captured event
      **/
      QEvent* getCloseEvent(void);

      /**
      @brief Returns the exit triggered status
      @return bool True if exit has been triggered
      **/
      bool closeTriggered(void);

      /**
      @brief Shuts down the main window
      @return void
      **/
      void shutdown(void);

      /**
      @brief Sets the map_initialized_ private variable
      @param mi [bool] The new value
      @return void
      **/
      void setMapInitialized(bool mi);

      /**
      @brief Unchecks the zoom in & out buttons when right click in map is pushed
      @return void
      **/
      void uncheckZoomButtons(void);

    //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief Qt slot that is called when the About tool button is pressed
      @return void
      **/
      void actionAboutTriggered(void);

      /**
      @brief Qt slot that is called when the Exit action is triggered
      @return void
      **/
      void actionExitTriggered(void);

      /**
      @brief Qt slot that is called when the LoadMap tool button is pressed
      @return void
      **/
      void actionLoadMapTriggered(void);

      /**
      @brief Qt slot that is called when the LoadSlamMap tool button is pressed
      @return void
      **/
      void actionLoadSlamMapTriggered(void);

      /**
      @brief Qt slot that is called when the zoom in tool button is pressed
      @return void
      **/
      void actionZoomInTriggered(void);

      /**
      @brief Qt slot that is called when the zoom out tool button is pressed
      @return void
      **/
      void actionZoomOutTriggered(void);

      /**
      @brief Qt slot that is called when the adjusted map visualization tool button is pressed
      @return void
      **/
      void actionAdjustedTriggered(void);

      /**
      @brief Qt slot that is called when the grid status has changed
      @return void
      **/
      void actionGridTriggered(void);

    //------------------------------------------------------------------------//
    Q_SIGNALS:

      /**
      @brief Qt signal that is emmited in GuiConnector::actionZoomInTriggered and connects to MapLoader::setCursorZoomIn
      @param state [bool] Toggle flag
      @return void
      **/
      void setZoomInCursor(bool state);

      /**
      @brief Qt signal that is emmited in GuiConnector::actionZoomOutTriggered and connects to MapLoader::setCursorZoomOut
      @param state [bool] Toggle flag
      @return void
      **/
      void setZoomOutCursor(bool state);

      /**
      @brief Qt signal that is emmited when the Adjust map button is pressed
      @param state [bool] Toggle flag
      @return void
      **/
      void setAdjustedCursor(bool state);

      /**
      @brief Signal emmited on exit pressed
      @return void
      **/
      void guiExitEvent(void);

  };
}

#endif
