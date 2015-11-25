cmake_minimum_required(VERSION 2.8.3)
project(ogm_gui)

set(CMAKE_INCLUDE_CURRENT_DIR_ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  ogm_msgs
  ogm_server
  nav_msgs
  map_server
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
  UiTools
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
 ${Qt5Widgets_INCLUDE_DIS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5UiTools_INCLUDE_DIRS}
  )

link_directories(
  ${catkin_LIBRARY_DIRS}
  )

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(${Qt5Widgets_DEFINITIONS}
                ${Qt5Gui_DEFINITIONS}
                ${Qt5UiTools_DEFINITIONS}
                ${Qt5Core_DEFINITIONS})

#Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")


catkin_package(
  DEPENDS
    Qt5Core
    Qt5Widgets
    Qt5Gui
    QtUiTools
  CATKIN_DEPENDS
    roscpp
    roslib
    ogm_server
    ogm_msgs
    nav_msgs
    map_server
  INCLUDE_DIRS include
  LIBRARIES
)

#----------------------------------------------------------------------------------------------------#
set(QT_FORMS
  ui/map.ui
  ui/alignment.ui
  ui/mainWindow.ui
  ui/validation.ui
  )

QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_gui_controller.h)
QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_gui_connector.h)
QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_validation_connector.h)
QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_map_connector.h)
QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_alignment_connector.h)
QT_WRAP_CPP(ogm_gui_node SAMPLE_MOC_SRCS include/ogm_gui/ogm_map_item.h)

#----------------------------------------------------------------------------------------------------#

#~ set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

add_executable(ogm_gui_node
  src/ogm_gui/ogm_gui_node.cpp
  src/ogm_gui/ogm_gui_application.cpp
  src/ogm_gui/ogm_gui_connector.cpp
  src/ogm_gui/ogm_gui_loader.cpp
  src/ogm_gui/ogm_gui_controller.cpp
  src/ogm_gui/ogm_validation_connector.cpp
  src/ogm_gui/ogm_validation_loader.cpp
  src/ogm_gui/ogm_alignment_connector.cpp
  src/ogm_gui/ogm_alignment_loader.cpp
  src/ogm_gui/ogm_map_connector.cpp
  src/ogm_gui/ogm_map_loader.cpp
  src/ogm_gui/ogm_map_item.cpp
  src/ogm_gui/ogm_tools.cpp
   ${QT_FORMS_HPP} ${SAMPLE_MOC_SRCS}
)
# add_dependencies(ogm_gui_node ogm_msgs_gencpp )  # Wait for std messages to build
target_link_libraries(ogm_gui_node
  ${catkin_LIBRARIES} 
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5UiTools_LIBRARIES}
  ${Qt5Core_LIBRARIES}
)

# Install launch files
install(DIRECTORY launch resources
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

# Install excecutables
install(TARGETS ogm_gui_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

