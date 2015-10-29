/******************************************************************************
   OGM Validator - Occupancy Grid Map Validator
   Copyright (C) 2015 OGM Valiator
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

#ifndef OGM_APPLICATION
#define OGM_APPLICATION

#include <QDebug>
#include <QtWidgets/QApplication>

/**
@namespace ogm_gui
@brief The main namespace for OGM GUI
**/
namespace ogm_gui
{
  /**
  @class COgmApplication
  @brief Inherits QApplications. Created to overload the notify function.
  **/
  class COgmApplication:public QApplication
  {
    public:

      /**
      @brief Default contructor
      @param argc [int&] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      COgmApplication(int &argc,char **argv);

      /**
      @brief Called at every Qt event
      @param receiver [QObject*] The event receiver
      @param event [QEvent*] The event triggered
      @return bool : True if receiver was notified about event
      **/
      bool notify(QObject * receiver, QEvent * event);
  };

}

#endif
