cmake_minimum_required(VERSION 2.8.3)
project(ogm_communications)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

add_message_files(
    FILES
    MapsMsg.msg
    MapPublish.msg
    MapPose.msg
)

add_service_files(
    FILES
    LoadExternalMap.srv
    LoadExternalMaps.srv
    LoadMapsFromServer.srv
    GuiRequestEvaluation.srv
    GuiRequestStructuralEvaluation.srv
    ServerRequestEvaluation.srv
    StructuralEvaluation.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ogm_msgs
 CATKIN_DEPENDS
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
#  DEPENDS system_lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


