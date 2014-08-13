cmake_minimum_required(VERSION 2.8.3)
project(current_limit_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pal_control_msgs
  pal_ros_control
  pluginlib
  realtime_tools
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    pal_control_msgs
    pal_ros_control
    pluginlib
    realtime_tools
    roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} pal_control_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
