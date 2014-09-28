cmake_minimum_required(VERSION 2.8.3)
project(excel_move)

SET (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
SET (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_CMAKE_DIR}")

set (DMTX_FOUND FALSE)
find_package (DMTX)
find_package(OpenCV REQUIRED)

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
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization robot_state_publisher roscpp rospy xacro
  cv_bridge image_transport opencv2
  DEPENDS system_lib
)

include_directories(
  if (DMTX_FOUND)
    ${DMTX_INCLUDE_DIRS}
  endif()
  ${catkin_INCLUDE_DIRS}
)

add_library(moveit_to_known_position src/move_to_known_position.cpp)
add_library(moveit_joint src/move_joint.cpp)
add_library(move_to_carthesian_position src/move_to_carthesian_position.cpp)
add_library(open_gripper src/open_gripper.cpp)
add_library(close_gripper src/close_gripper.cpp)
add_library(scan_table src/scan_table.cpp)
add_library(scanning src/scanning.cpp)
add_library(scanning_sim src/scanning_sim.cpp)

add_executable(moveit_to_known_position_node src/move_to_known_position.cpp)
add_executable(moveit_joint_node src/move_joint.cpp)
add_executable(move_to_carthesian_position_node src/move_to_carthesian_position.cpp)
add_executable(open_gripper_node src/open_gripper.cpp)
add_executable(close_gripper_node src/close_gripper.cpp)
add_executable(scan_table_node src/scan_table.cpp)
add_executable(scanning_sim_node src/scanning_sim.cpp)
if (DMTX_FOUND)
  add_executable(scanning_node src/scanning.cpp)
endif()

target_link_libraries(moveit_to_known_position_node ${catkin_LIBRARIES})
target_link_libraries(moveit_joint_node ${catkin_LIBRARIES})
target_link_libraries(move_to_carthesian_position_node ${catkin_LIBRARIES})
target_link_libraries(open_gripper_node ${catkin_LIBRARIES})
target_link_libraries(close_gripper_node ${catkin_LIBRARIES})
target_link_libraries(scan_table_node ${catkin_LIBRARIES})
target_link_libraries(scanning_sim_node ${catkin_LIBRARIES})
if (DMTX_FOUND)
  target_link_libraries(scanning_node ${catkin_LIBRARIES}    
    ${OpenCV_LIBRARIES}
    ${DMTX_LIBRARIES}
    )
endif()
