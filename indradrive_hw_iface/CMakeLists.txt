cmake_minimum_required(VERSION 2.8.3)
project(indradrive_hw_iface)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  controller_manager
  roscpp
  std_msgs
  urdf
)

catkin_python_setup()

###########
## Build ##
###########

find_library(ECAT_LIBRARY
  NAMES ethercat
  PATHS /opt/etherlab/lib
)

find_library(XENOMAI_LIBRARY
  NAMES xenomai
  PATHS /usr/xenomai/lib
)

set(EXPORT_LIBRARIES idcs_test_vel_ctrl)

if( NOT (ECAT_LIBRARY MATCHES "ECAT_LIBRARY-NOTFOUND") )
  set(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/etherlab/include)
  set(DEPENDS_LIBRARIES ${ECAT_LIBRARY})
  set(EXPORT_LIBRARIES ${EXPORT_LIBRARIES} idcs_hw_vel_ctrl)
  set(CTRL_LINK_LIBS ${CTRL_LINK_LIBS} ${ECAT_LIBRARY} rt m)

  if( NOT (XENOMAI_LIBRARY MATCHES "XENOMAI_LIBRARY-NOTFOUND" ) )
    message( "Xenomai found, also installing xeno realtime binaries" )
    set(INCLUDE_DIRS ${INCLUDE_DIRS} /usr/xenomai/include)
    set(DEPENDS_LIBRARIES ${DEPENDS_LIBRARIES} xenomai native rtdm)
    set(CTRL_LINK_LIBS ${CTRL_LINK_LIBS} xenomai native rtdm)
    link_directories(/usr/xenomai/lib)
  else()
    message( "Xenomai libary not found so RT driver will not be built" )
  endif()

else()
  message( "Ethercat master libary not found so hardware indradrive controller will not be built." )
endif()

set(INST_TARGETS ${EXPORT_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${EXPORT_LIBRARIES}
  DEPENDS ${DEPENDS_LIBRARIES}
  CATKIN_DEPENDS urdf controller_interface hardware_interface roscpp
)
set(INCLUDE_DIRS ${INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})
set(CTRL_LINK_LIBS ${CTRL_LINK_LIBS} ${catkin_LIBRARIES})
include_directories(${INCLUDE_DIRS})

add_library(idcs_test_vel_ctrl src/idcs_robot_hw.cpp)
target_link_libraries(idcs_test_vel_ctrl ${CTRL_LINK_LIBS})

add_executable(test_vel_ctrl_man src/vel_ctrl_man.cpp)
target_link_libraries(test_vel_ctrl_man idcs_test_vel_ctrl ${CTRL_LINK_LIBS})
set_target_properties(test_vel_ctrl_man PROPERTIES COMPILE_DEFINITIONS "TEST_CTRL")
set(INST_TARGETS ${INST_TARGETS} test_vel_ctrl_man)

if( NOT (ECAT_LIBRARY MATCHES "ECAT_LIBRARY-NOTFOUND") )
  
  ## Declare a cpp library
  add_library(idcs_hw_vel_ctrl
    src/idcs_robot_hw.cpp
    src/ethercat_ctrl.cpp
    src/vel_ec_ctrl.cpp
  )
  target_link_libraries(idcs_hw_vel_ctrl ${CTRL_LINK_LIBS})
  
  add_executable(vel_ctrl_man src/vel_ctrl_man.cpp)
  target_link_libraries(vel_ctrl_man idcs_hw_vel_ctrl ${CTRL_LINK_LIBS})
  set(INST_TARGETS ${INST_TARGETS} vel_ctrl_man)
  
  if( NOT (XENOMAI_LIBRARY MATCHES "XENOMAI_LIBRARY-NOTFOUND" ) )
    add_executable(vel_ctrl_man_xeno src/vel_ctrl_man.cpp)
    set_target_properties(vel_ctrl_man_xeno PROPERTIES COMPILE_DEFINITIONS "XENOMAI_REALTIME")
    target_link_libraries(vel_ctrl_man_xeno idcs_hw_vel_ctrl ${CTRL_LINK_LIBS})
    set(INST_TARGETS ${INST_TARGETS} vel_ctrl_man_xeno)
  endif()
endif()

  
## Mark executables and/or libraries for installation
install(TARGETS ${INST_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
