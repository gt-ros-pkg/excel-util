cmake_minimum_required(VERSION 2.8.3)
project(excel_hw_iface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  ur_ctrl_server
  ur_ctrl_client
  ur_controllers
  simple_message
  joint_limits_interface
  indradrive_hw_iface
  roscpp
  ur_ctrl_client
  urdf
)

catkin_python_setup()

###########
## Build ##
###########

find_library(INDRADRIVE_HW_LIBRARY 
  NAMES idcs_hw_vel_ctrl
)

find_library(XENOMAI_LIBRARY
  NAMES xenomai
  PATHS /usr/xenomai/lib
)

if( NOT (INDRADRIVE_HW_LIBRARY MATCHES "INDRADRIVE_HW_LIBRARY-NOTFOUND") )
  set(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/etherlab/include)
  set(DEPENDS_LIBRARIES ${ECAT_LIBRARY})
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

catkin_package(
  CATKIN_DEPENDS indradrive_hw_iface ur_ctrl_server ur_ctrl_client
)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set(CTRL_LINK_LIBS urdf ${catkin_LIBRARIES} ${CTRL_LINK_LIBS})
include_directories(${INCLUDE_DIRS})

add_executable(test_excel_ctrl_man src/excel_ctrl_man.cpp)
target_link_libraries(test_excel_ctrl_man idcs_test_vel_ctrl ${CTRL_LINK_LIBS} ${catkin_LIBRARIES})
set_target_properties(test_excel_ctrl_man PROPERTIES COMPILE_DEFINITIONS "TEST_CTRL")
message(${CTRL_LINK_LIBS})

if( NOT (INDRADRIVE_HW_LIBRARY MATCHES "INDRADRIVE_HW_LIBRARY-NOTFOUND") )
  if( XENOMAI_LIBRARY MATCHES "XENOMAI_LIBRARY-NOTFOUND")
    add_executable(excel_ctrl_man src/excel_ctrl_man.cpp)
    target_link_libraries(excel_ctrl_man ${CTRL_LINK_LIBS})
  else()
    add_executable(excel_ctrl_man_xeno src/excel_ctrl_man.cpp)
    set_target_properties(excel_ctrl_man_xeno PROPERTIES COMPILE_DEFINITIONS "XENOMAI_REALTIME")
    target_link_libraries(excel_ctrl_man_xeno ${CTRL_LINK_LIBS})
  endif()
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS excel_hw_iface excel_hw_iface_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_excel_hw_iface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
