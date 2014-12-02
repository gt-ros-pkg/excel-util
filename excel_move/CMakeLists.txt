cmake_minimum_required(VERSION 2.8.3)
project(excel_move)

SET (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
SET (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_CMAKE_DIR}")

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
  actionlib_msgs
)

add_action_files(
  FILES
  Scanning.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization robot_state_publisher roscpp rospy xacro
  cv_bridge image_transport 
  DEPENDS system_lib
)

set (EXCEL_MOVE_INCLUDES include ${catkin_INCLUDE_DIRS})

if (DMTX_FOUND)
  set (EXCEL_MOVE_INCLUDES ${EXCEL_MOVE_INCLUDES} ${DMTX_INCLUDE_DIRS})
endif()

include_directories(${EXCEL_MOVE_INCLUDES})

add_executable(moveit_to_known_position_node src/move_to_known_position.cpp)
add_executable(moveit_joint_node src/move_joint.cpp)
add_executable(move_to_carthesian_position_node src/move_to_carthesian_position.cpp)
add_executable(open_gripper_node src/open_gripper.cpp)
add_executable(close_gripper_node src/close_gripper.cpp)
add_executable(scan_table_node src/scan_table.cpp)
add_executable(scanning_sim_node src/scanning_sim.cpp)
add_executable(back_n_forth_node src/back_n_forth.cpp)
add_executable(dodge_human_node src/dodge_human.cpp)
add_executable(move_to_joint_positions_node src/move_to_joint_positions.cpp)

target_link_libraries(moveit_to_known_position_node ${catkin_LIBRARIES})
target_link_libraries(moveit_joint_node ${catkin_LIBRARIES})
target_link_libraries(move_to_carthesian_position_node ${catkin_LIBRARIES})
target_link_libraries(open_gripper_node ${catkin_LIBRARIES})
target_link_libraries(close_gripper_node ${catkin_LIBRARIES})
target_link_libraries(scan_table_node ${catkin_LIBRARIES})
target_link_libraries(scanning_sim_node ${catkin_LIBRARIES})
target_link_libraries(back_n_forth_node ${catkin_LIBRARIES})
target_link_libraries(dodge_human_node ${catkin_LIBRARIES})
target_link_libraries(move_to_joint_positions_node ${catkin_LIBRARIES})

target_link_libraries(move_to_joint_positions_node yaml-cpp)

if (DMTX_FOUND)
  add_library(scanning src/scanning.cpp)

  add_executable(scanning_node src/scanning_node.cpp)
  add_executable(scanning_action_server src/scanning_action_server.cpp)

  add_dependencies(scanning_action_server
    ${excel_move_EXPORTED_TARGETS}
  )

  target_link_libraries(scanning 
    ${OpenCV_LIBRARIES}
    ${DMTX_LIBRARIES}
  )

  target_link_libraries(scanning_node 
    scanning
    ${catkin_LIBRARIES}    
  )
  target_link_libraries(scanning_action_server 
    scanning
    ${catkin_LIBRARIES}    
  )
endif()

