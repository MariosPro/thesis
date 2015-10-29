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

#include "ogm_gui/ogm_alignment_loader.h"

namespace ogm_gui
{

  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CAlignmentLoader::CAlignmentLoader(int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    addAlignmentIcons();
  }

  /**
  @brief Default destructor
  @return void
  **/
  CAlignmentLoader::~CAlignmentLoader(void)
  {

  }

  /**
  @brief Add the icons on the alignment push buttons
  @return void
  **/
  void CAlignmentLoader::addAlignmentIcons()
  {
    moveUppushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_up.png")).c_str())));

    moveDownpushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_down.png")).c_str())));

    moveRightpushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_right.png")).c_str())));

    moveLeftpushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/translate_left.png")).c_str())));

    rotateRightpushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/rotate_right.png")).c_str())));

    rotateLeftpushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/rotate_left.png")).c_str())));

    scalepushButton->setIcon(QIcon(QString::fromUtf8((
      ogm_gui_tools::getRosPackagePath("ogm_gui") +
      std::string("/resources/images/adjusted.png")).c_str())));

  }
}

