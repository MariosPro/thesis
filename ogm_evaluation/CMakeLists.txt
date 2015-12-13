cmake_minimum_required(VERSION 2.8.3)
project(ogm_evaluation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  ogm_msgs
  geometry_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
  ogm_evaluator
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    ogm_msgs
    geometry_msgs
    nav_msgs
)

######################### Metrics ######################################
add_library(ogm_metric_base src/ogm_evaluation/ogm_evaluation_metrics/metric_base.cpp)
target_link_libraries(ogm_metric_base ${catkin_LIBRARIES})

add_library (ogm_evaluator src/ogm_evaluation/ogm_evaluation.cpp)
add_dependencies(ogm_evaluator ogm_msgs_gencpp) #wait for ogm_msgs to be build
target_link_libraries(ogm_evaluator ${catkin_LIBRARIES})

add_executable(ogm_evaluation src/ogm_evaluation/ogm_evaluation_node.cpp)
add_dependencies(ogm_evaluation ogm_msgs_gencpp) # wait for ogm_msgs to be build
target_link_libraries(ogm_evaluation ${catkin_LIBRARIES} ogm_evaluator)

# Install libraries
install(TARGETS
    ogm_metric_base
    ogm_evaluator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install excecutables
install(TARGETS ogm_evaluation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
