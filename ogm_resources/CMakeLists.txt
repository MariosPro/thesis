cmake_minimum_required(VERSION 2.8.3)
project(ogm_resources)

find_package(catkin REQUIRED)

catkin_package()

# Install files
install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


