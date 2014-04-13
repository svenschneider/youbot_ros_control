cmake_minimum_required(VERSION 2.8.3)
project(yrc_youbot)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pluginlib
    yrc_hardware_interface
)

find_package(youbot_driver REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(oodl_ros_control
  src/oodl_ros_control.cpp
)

target_link_libraries(oodl_ros_control
  ${catkin_LIBRARIES}
  ${youbot_driver_LIBRARIES}
)

install(TARGETS oodl_ros_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/oodl_ros_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yrc_youbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()