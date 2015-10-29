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

#include "ogm_gui/ogm_alignment_connector.h"

namespace ogm_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CAlignmentConnector::CAlignmentConnector(int argc, char **argv):
    QObject(),
    loader(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    QObject::connect(
        loader.moveUppushButton, SIGNAL(pressed()),
        this, SLOT(moveUppushButtonTriggered()));
    QObject::connect(
        loader.moveDownpushButton, SIGNAL(pressed()),
        this, SLOT(moveDownpushButtonTriggered()));
    QObject::connect(
        loader.moveLeftpushButton, SIGNAL(pressed()),
        this, SLOT(moveLeftpushButtonTriggered()));
    QObject::connect(
        loader.moveRightpushButton, SIGNAL(pressed()),
        this, SLOT(moveRightpushButtonTriggered()));
    QObject::connect(
        loader.rotateLeftpushButton, SIGNAL(pressed()),
        this, SLOT(rotateLeftpushButtonTriggered()));
    QObject::connect(
        loader.rotateRightpushButton, SIGNAL(pressed()),
        this, SLOT(rotateRightpushButtonTriggered()));
    QObject::connect(
        loader.scalepushButton, SIGNAL(pressed()),
        this, SLOT(scalepushButtonTriggered()));

  }

  /**
  @brief Qt slot that is called when the moveUp pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::moveUppushButtonTriggered(void)
  {
    Q_EMIT moveUpMapPressed();
  }

  /**
  @brief Qt slot that is called when the moveDown pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::moveDownpushButtonTriggered(void)
  {
    Q_EMIT moveDownMapPressed();
  }

  /**
  @brief Qt slot that is called when the moveLeft pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::moveLeftpushButtonTriggered(void)
  {
    Q_EMIT moveLeftMapPressed();
  }

  /**
  @brief Qt slot that is called when the moveRight pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::moveRightpushButtonTriggered(void)
  {
    Q_EMIT moveRightMapPressed();
  }

  /**
  @brief Qt slot that is called when the rotateLeft pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::rotateLeftpushButtonTriggered(void)
  {
    Q_EMIT rotateLeftMapPressed();
  }

  /**
  @brief Qt slot that is called when the rotateRight pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::rotateRightpushButtonTriggered(void)
  {
    Q_EMIT rotateRightMapPressed();
  }

  /**
  @brief Qt slot that is called when the scale pushButton is triggered
  @return void
  **/
  void CAlignmentConnector::scalepushButtonTriggered(void)
  {
    Q_EMIT scaleMapPressed();
  }

  /**
  @brief Returns the CAlignmentLoader object
  @return QWidget*
  **/
  QWidget* CAlignmentConnector::getLoader(void)
  {
    return static_cast<QWidget *>(&loader);
  }
}

