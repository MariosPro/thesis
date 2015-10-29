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

#ifndef OGM_ALIGNMENT_LOADER
#define OGM_ALIGNMENT_LOADER

#include "ui_alignment.h"
#include "ogm_gui/ogm_tools.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class CAlignmentLoader
  @brief Implements the low level Qt functionalities of the Validation widget. Inherits from Ui_validation (generated from an ui file) and QWidget.
  **/
  class CAlignmentLoader :
    public QWidget,
    public Ui_alignmentWidget
  {
    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char **  argv_;

    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CAlignmentLoader(int argc, char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CAlignmentLoader(void);

      /**
      @brief Add the icons on the alignment push buttons
      @return void
      **/
      void addAlignmentIcons();

  };
}

#endif
