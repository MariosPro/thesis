cmake_minimum_required(VERSION 2.8.3)
project(ogm_evaluation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  ogm_msgs
  geometry_msgs
  nav_msgs
)
find_package(OpenCV REQUIRED nonfree)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
  ogm_evaluation
  CATKIN_DEPENDS
    roscpp
    roslib
    cv_bridge
    ogm_msgs
    geometry_msgs
    nav_msgs
)

######################### Metrics ######################################
add_library(ogm_evaluation_utils
            src/ogm_evaluation/ogm_evaluation_utils/affine2DEstimator.cpp
            src/ogm_evaluation/ogm_evaluation_utils/maps_alignment.cpp)
target_link_libraries(ogm_evaluation_utils ${catkin_LIBRARIES})


add_library(ogm_custom_descriptors 
            src/ogm_evaluation/ogm_evaluation_descriptors/radius_statistics_descriptor.cpp
            src/ogm_evaluation/ogm_evaluation_descriptors/circle_intersection_descriptor.cpp
            src/ogm_evaluation/ogm_evaluation_descriptors/mean_rays_descriptor.cpp)
target_link_libraries(ogm_custom_descriptors)

add_library(ogm_metric_base src/ogm_evaluation/ogm_evaluation_metrics/metric_base.cpp)
target_link_libraries(ogm_metric_base ${catkin_LIBRARIES})

add_library(ogm_omse_metric src/ogm_evaluation/ogm_evaluation_metrics/omse.cpp)
target_link_libraries(ogm_omse_metric ${catkin_LIBRARIES} ogm_metric_base)

add_library(ogm_cmse_metric src/ogm_evaluation/ogm_evaluation_metrics/cmse.cpp)
target_link_libraries(ogm_cmse_metric ${catkin_LIBRARIES} ogm_metric_base)

add_library(ogm_descriptor_factory src/ogm_evaluation/ogm_evaluation_descriptors/descriptor_factory.cpp)
target_link_libraries(ogm_descriptor_factory ${catkin_LIBRARIES} ogm_custom_descriptors)

add_library(ogm_feature_metrics src/ogm_evaluation/ogm_evaluation_metrics/feature_metrics.cpp)
target_link_libraries(ogm_feature_metrics 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ogm_metric_base
                      ogm_descriptor_factory
                      ogm_evaluation_utils)

add_library(ogm_metric_factory src/ogm_evaluation/ogm_metric_factory.cpp)
target_link_libraries(ogm_metric_factory
                      ${catkin_LIBRARIES}
                      ogm_omse_metric
                      ogm_cmse_metric
                      ogm_feature_metrics
                      )

add_library (ogm_evaluation src/ogm_evaluation/ogm_evaluation.cpp)
add_dependencies(ogm_evaluation ogm_msgs_gencpp) #wait for ogm_msgs to be build
target_link_libraries(ogm_evaluation ${catkin_LIBRARIES} ogm_metric_factory)

add_executable(ogm_evaluation_node src/ogm_evaluation/ogm_evaluation_node.cpp)
add_dependencies(ogm_evaluation_node ogm_msgs_gencpp) # wait for ogm_msgs to be build
target_link_libraries(ogm_evaluation_node ${catkin_LIBRARIES} ogm_evaluation)

add_executable(map_saver src/ogm_evaluation/ogm_evaluation_utils/map_saver.cpp)
target_link_libraries(map_saver ${catkin_LIBRARIES} )
# Install libraries
install(TARGETS
    ogm_metric_base
    ogm_omse_metric
    ogm_cmse_metric
    ogm_custom_descriptors
    ogm_descriptor_factory
    ogm_feature_metrics
    ogm_metric_factory
    ogm_evaluation_utils
    ogm_evaluation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install excecutables
install(TARGETS ogm_evaluation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
