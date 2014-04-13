cmake_minimum_required(VERSION 2.8.3)
project(yrc_hardware_interface)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hardware_interface
)

find_package(youbot_driver REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${youbot_driver_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    hardware_interface
  DEPENDS
    ${youbot_driver_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)