cmake_minimum_required(VERSION 2.8.3)
project(ogm_server)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    sensor_msgs
    ogm_msgs
    map_server
)

set(CMAKE_BUILD_TYPE Release)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ogm_map_loader
  CATKIN_DEPENDS
    roscpp
    tf
    nav_msgs
)

link_directories(${catkin_LIBRARY_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS})

add_library(ogm_map_loader src/map_loader.cpp)
target_link_libraries(ogm_map_loader
    image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

add_library(ogm_map_server src/map_server.cpp)
target_link_libraries(ogm_map_server
    ogm_map_loader
    ${catkin_LIBRARIES}
)

add_library(ogm_server src/ogm_server.cpp)
add_dependencies(ogm_server ogm_msgs_gencpp) # wait for ogm_msgs to be build
target_link_libraries(ogm_server
	ogm_map_server
	${catkin_LIBRARIES}
)

add_executable(ogm_server_node src/ogm_server_node.cpp)
target_link_libraries(ogm_server_node
	ogm_server
	${catkin_LIBRARIES}
)

add_executable(ogm_load_map src/map_loader_node.cpp)
add_dependencies(ogm_load_map ogm_msgs_gencpp) # wait for ogm_msgs to be build
target_link_libraries(ogm_load_map
	ogm_map_loader
	${catkin_LIBRARIES}
)

# Install libraries
install(TARGETS 
    ogm_map_loader
    ogm_map_server
    ogm_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install excecutables
install(TARGETS
    ogm_server_node
    ogm_load_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES include/${PROJECT_NAME}/map_loader.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
