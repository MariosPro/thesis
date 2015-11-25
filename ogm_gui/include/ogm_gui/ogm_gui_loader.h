/******************************************************************************
   OGM - Occupancy Grid Map Validator
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

  Authors:
   * Marios Protopapas, protopapasmarios@gmail.com
   * Manos Tsardoulias, etsardou@gmail.com
******************************************************************************/

#ifndef OGM_GUI_LOADER
#define OGM_GUI_LOADER

#include "ui_mainWindow.h"
#include "ogm_gui/ogm_tools.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/ 
namespace ogm_gui
{
  /**
  @class GuiConnector
  @brief Implements the low level Qt functionalities of the main window. Inherits from Ui_MainWindow (generated from an ui file) and QMainWindow. 
  **/ 
  class CGuiLoader : public Ui_MainWindow, public QMainWindow
  {
    //------------------------------------------------------------------------//
    private:
      //!< The number of input arguments
      int   argc_;
      //!< The input arguments
      char**  argv_;
      
      //!< True if the exit signal was emmited
      bool   close_signal_;
      //!< The exit event (when occurs)
      QCloseEvent   *event_;
      
    //------------------------------------------------------------------------//
    public:

      //!< The action of properties button 
      /* QAction *actionProperties; */

      //!< The action of toggle grid button
      QAction *actionGrid;

      //!< The action of loading a ground thruth map button
      QAction *actionLoadMap;

      //!< The action of loading a SLAM-produced map button
      QAction *actionLoadSlamMap;

      //!< The action of zooming in button
      QAction *actionZoomIn;

      //!< The action of zooming out button
      QAction *actionZoomOut;

      //!< The action of map adjusted button
      QAction *actionAdjusted;

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CGuiLoader(int argc,char **argv);
      
      /**
      @brief Overloading of closeEvent function from QMainWindow
      @param event [QCloseEvent*] The exit event
      @return void
      **/
      void closeEvent(QCloseEvent *event);
      
      /**
      @brief Adds the tool buttons in the main window toolbar
      @return void
      **/
      void addToolbarIcons(void);
      
      /**
      @brief Returns the exit event
      @return QEvent* 
      **/
      QEvent* getCloseEvent(void);
      
      /**
      @brief Returns true if a close event was triggered
      @return bool
      **/
      bool closeTriggered(void);
      
      /**
      @brief Shuts down the main window
      @return void
      **/
      void shutdown(void);
      
  };
  
}

#endif
