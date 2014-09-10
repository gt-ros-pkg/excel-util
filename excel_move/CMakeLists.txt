cmake_minimum_required(VERSION 2.8.3)
project(excel_move)

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
  control_msgs
  genmsg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(moveit_to_known_position src/move_to_known_position.cpp)
add_library(moveit_joint src/move_joint.cpp)
add_library(move_to_carthesian_position src/move_to_carthesian_position.cpp)

add_executable(moveit_to_known_position_node src/move_to_known_position.cpp)
add_executable(moveit_joint_node src/move_joint.cpp)
add_executable(move_to_carthesian_position_node src/move_to_carthesian_position.cpp)

target_link_libraries(moveit_to_known_position_node ${catkin_LIBRARIES})
target_link_libraries(moveit_joint_node ${catkin_LIBRARIES})
target_link_libraries(move_to_carthesian_position_node ${catkin_LIBRARIES})

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES excel_move
#  CATKIN_DEPENDS geometry_msgs joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization robot_state_publisher roscpp rospy xacro
#  DEPENDS system_lib
#)
