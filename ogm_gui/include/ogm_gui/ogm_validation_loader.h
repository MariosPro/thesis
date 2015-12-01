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

#ifndef OGM_VALIDATION_LOADER
#define OGM_VALIDATION_LOADER

#include "ui_validation.h"
#include "ogm_gui/ogm_tools.h"

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class CValidationLoader
  @brief Implements the low level Qt functionalities of the Validation TabWidget. Inherits from Ui_validation (generated from an ui file) and QWidget.
  **/
  class CValidationLoader : public QWidget, public Ui_validationWidget
  {
    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char **  argv_;

    public:

      //!< Tree item : map Root
      QTreeWidgetItem mapInfo;
      //!< Tree item : mapEvaluation Root
      QTreeWidgetItem mapEvaluation;
      //!< Tree item : pathEvaluation Root
      QTreeWidgetItem pathEvaluation;
      //!< Tree item: image general metrics
      QTreeWidgetItem metrics;
      //!< Tree item: Structural Evaluation
      QTreeWidgetItem structEvaluation;
      //!< Tree item: Feature Evaluation
      QTreeWidgetItem featEvaluation;

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CValidationLoader(int argc, char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CValidationLoader(void);

      /**
      @brief Deletes the information tree
      @return void
      **/
      void deleteTree(void);

      /**
      @brief Deletes a specific tree node. Recursive function.
      @param item [QTreeWidgetItem*] The item to be deleted
      @return void
      **/
      void deleteTreeNode(QTreeWidgetItem *item);

      /**
      @brief Updates the information tree according to the specific map
      @param width [float] The map width
      @param height [float] The map height
      @param ocgd [float] The map resolution (m/pixel)
      @return void
      **/
      void updateMapInfo(float width, float height, float ocgd, bool groundTruth);

      /**
      @brief Autoresizes the columns according to the visible contents
      @return void
      **/
      void autoResizeColumns(void);

  };
}

#endif
