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

#ifndef OGM_ALIGNMENT_CONNECTOR
#define OGM_ALIGNMENT_CONNECTOR

#include "ogm_gui/ogm_alignment_loader.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class CAlignmentConnector
  @brief Serves the Qt events of the maps alignment widget. Inherits from QObject
  **/
  class CAlignmentConnector :
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

      //!< Object of CAlignmentLoader
      CAlignmentLoader loader;

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CAlignmentConnector(int argc, char **argv);

      /**
      @brief Returns the CAlignmentLoader object
      @return QWidget*
      **/
      QWidget* getLoader(void);


    //------------------------------------------------------------------------//
    public Q_SLOTS:
      /**
      @brief Qt slot that is called when the moveUp pushButton is triggered
      @return void
      **/
      void moveUppushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the moveDown pushButton is triggered
      @return void
      **/
      void moveDownpushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the moveLeft pushButton is triggered
      @return void
      **/
      void moveLeftpushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the moveRight pushButton is triggered
      @return void
      **/
      void moveRightpushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the rotateLeft pushButton is triggered
      @return void
      **/
      void rotateLeftpushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the rotateRight pushButton is triggered
      @return void
      **/
      void rotateRightpushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the scale pushButton is triggered
      @return void
      **/
      void scalepushButtonTriggered(void);



    //------------------------------------------------------------------------//
    Q_SIGNALS:

      /**
      @brief Is emmited when the moveUp Button has been pressed.
       Connects to the CGuiConntroller::moveUpMap slot
      @return void
      **/
      void moveUpMapPressed(void);

      /**
      @brief Is emmited when the moveDown Button has been pressed.
       Connects to the CGuiConntroller::moveDownMap slot
      @return void
      **/
      void moveDownMapPressed(void);

      /**
      @brief Is emmited when the moveLeft Button has been pressed.
       Connects to the CGuiConntroller::moveLeftMap slot
      @return void
      **/
      void moveLeftMapPressed(void);

      /**
      @brief Is emmited when the moveRight Button has been pressed.
       Connects to the CGuiConntroller::moveRightMap slot
      @return void
      **/
      void moveRightMapPressed(void);

      /**
      @brief Is emmited when the rotateLeft Button has been pressed.
       Connects to the CGuiConntroller::rotateLeftMap slot
      @return void
      **/
      void rotateLeftMapPressed(void);

      /**
      @brief Is emmited when the rotateRight Button has been pressed.
       Connects to the CGuiConntroller::rotateRightMap slot
      @return void
      **/
      void rotateRightMapPressed(void);

      /**
      @brief Is emmited when the scale Button has been pressed.
       Connects to the CGuiConntroller::scaleMap slot
      @return void
      **/
      void scaleMapPressed(void);

  };
}
#endif



