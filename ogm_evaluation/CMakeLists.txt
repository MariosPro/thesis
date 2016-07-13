cmake_minimum_required(VERSION 2.8.3)
project(ogm_evaluation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  ogm_communications
  geometry_msgs
  nav_msgs
  sensor_msgs
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
    ogm_communications
    geometry_msgs
    nav_msgs
    sensor_msgs
)

######################### Metrics ######################################
add_library(${PROJECT_NAME}_utils
            src/${PROJECT_NAME}/ogm_evaluation_utils/affine2DEstimator.cpp
            src/${PROJECT_NAME}/ogm_evaluation_utils/map_utils.cpp)
add_dependencies(${PROJECT_NAME}_utils 
                ogm_communications_gencpp 
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
                ) #wait for ogm_communications to be build

target_link_libraries(${PROJECT_NAME}_utils ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_alignment
            src/${PROJECT_NAME}/ogm_evaluation_utils/maps_alignment.cpp)

add_dependencies(${PROJECT_NAME}_alignment
                ogm_communications_gencpp 
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
                ) #wait for ogm_communications to be build

target_link_libraries(${PROJECT_NAME}_alignment
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_utils)

add_library(ogm_custom_descriptors 
            src/${PROJECT_NAME}/ogm_evaluation_descriptors/radius_statistics_descriptor.cpp
            src/${PROJECT_NAME}/ogm_evaluation_descriptors/circle_intersection_descriptor.cpp
            src/${PROJECT_NAME}/ogm_evaluation_descriptors/mean_rays_descriptor.cpp)
target_link_libraries(ogm_custom_descriptors ${catkin_LIBRARIES})

add_library(ogm_metric_base src/${PROJECT_NAME}/ogm_evaluation_metrics/metric_base.cpp)

target_link_libraries(ogm_metric_base ${catkin_LIBRARIES})

add_library(ogm_omse_metric src/${PROJECT_NAME}/ogm_evaluation_metrics/omse.cpp)

target_link_libraries(ogm_omse_metric 
                      ${catkin_LIBRARIES}
                      ogm_metric_base
                      ${PROJECT_NAME}_utils
                      ${PROJECT_NAME}_alignment)

add_library(ogm_descriptor_factory src/${PROJECT_NAME}/ogm_evaluation_descriptors/descriptor_factory.cpp)

target_link_libraries(ogm_descriptor_factory ${catkin_LIBRARIES} ogm_custom_descriptors)

add_library(ogm_feature_metrics src/${PROJECT_NAME}/ogm_evaluation_metrics/feature_metrics.cpp)

add_dependencies(ogm_feature_metrics 
                ogm_communications_gencpp
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
) #wait for ogm_communications to be build
target_link_libraries(ogm_feature_metrics 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ogm_metric_base
                      ogm_descriptor_factory
                      ${PROJECT_NAME}_utils
                      ${PROJECT_NAME}_alignment)

add_library(ogm_metric_factory src/${PROJECT_NAME}/ogm_metric_factory.cpp)
target_link_libraries(ogm_metric_factory
                      ${catkin_LIBRARIES}
                      ogm_omse_metric
                      ogm_feature_metrics
                      )

add_library (${PROJECT_NAME} src/ogm_evaluation/ogm_evaluation.cpp)
add_dependencies(${PROJECT_NAME}
                ogm_communications_gencpp
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
) #wait for ogm_communications to be build
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ogm_metric_factory)

add_executable(${PROJECT_NAME}_node src/ogm_evaluation/ogm_evaluation_node.cpp)
add_dependencies(${PROJECT_NAME}_node 
                ogm_communications_gencpp
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
) #wait for ogm_communications to be build

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ogm_evaluation)

add_executable(map_saver src/${PROJECT_NAME}/ogm_evaluation_utils/map_saver.cpp)

target_link_libraries(map_saver ${catkin_LIBRARIES} )

# Install libraries
install(TARGETS
    ogm_metric_base
    ogm_omse_metric
    ogm_custom_descriptors
    ogm_descriptor_factory
    ogm_feature_metrics
    ogm_metric_factory
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install excecutables
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
