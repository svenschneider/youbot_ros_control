cmake_minimum_required(VERSION 2.8.3)
project(yrc_base)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pluginlib
    hardware_interface
    yrc_hardware_interface
)

find_package(Boost REQUIRED)
find_package(youbot_driver REQUIRED)

catkin_package(
  LIBRARIES yrc_base
  CATKIN_DEPENDS
    roscpp
    pluginlib
    hardware_interface
    yrc_hardware_interface
  DEPENDS
    ${youbot_driver_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${youbot_driver_INCLUDE_DIRS}
)

add_library(yrc_base
  src/yrc_base.cpp
)

install(TARGETS yrc_base
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yrc_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()