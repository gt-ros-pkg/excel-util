cmake_minimum_required(VERSION 2.8.3)
project(excel_moveit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joint_state_publisher
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_visualization
  robot_state_publisher
  roscpp
  rospy
  xacro
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Human.msg
  Humans.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES excel_moveit
 CATKIN_DEPENDS joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization robot_state_publisher geometry_msgs roscpp rospy xacro
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(publish_humans src/publish_humans.cpp)
add_library(listener_humans src/listener_humans.cpp)
add_library(moveit_to_known_position src/move_to_known_position.cpp)
add_library(moveit_joint src/move_joint.cpp)

## Declare a cpp executable
add_executable(publish_humans_node src/publish_humans.cpp)
add_executable(listener_humans_node src/listener_humans.cpp)
add_executable(moveit_to_known_position_node src/move_to_known_position.cpp)
add_executable(moveit_joint_node src/move_joint.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(publish_humans_node ${catkin_LIBRARIES})
target_link_libraries(listener_humans_node ${catkin_LIBRARIES})
target_link_libraries(moveit_to_known_position_node ${catkin_LIBRARIES})
target_link_libraries(moveit_joint_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
