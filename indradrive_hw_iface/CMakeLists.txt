cmake_minimum_required(VERSION 2.8.3)
project(indradrive_hw_iface)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  controller_manager
  roscpp
)

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

if( NOT (ECAT_LIBRARY MATCHES "ECAT_LIBRARY-NOTFOUND") )

  if( XENOMAI_LIBRARY MATCHES "XENOMAI_LIBRARY-NOTFOUND" )

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES idcs_hw_vel_ctrl
      DEPENDS expat ${ECAT_LIBRARY}
    #  CATKIN_DEPENDS controller_interface hardware_interface roscpp
    )
  else()

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES idcs_hw_vel_ctrl
      DEPENDS expat ${ECAT_LIBRARY} xenomai native rtdm
    #  CATKIN_DEPENDS controller_interface hardware_interface roscpp
    )
  endif()

  include_directories(
    include
    /opt/etherlab/include
    ${catkin_INCLUDE_DIRS}
  )
  
  ## Declare a cpp library
  add_library(idcs_hw_vel_ctrl
    src/cs_robot_hw.cpp
    src/ethercat_ctrl.cpp
    src/vel_ec_ctrl.cpp
  )
  
  target_link_libraries(idcs_hw_vel_ctrl
    ${catkin_LIBRARIES}
    ${ECAT_LIBRARY}
    expat
    rt
    m
  )
  
  ## Declare a cpp executable
  add_executable(idcs_ctrl_man src/cs_ctrl_man.cpp)
  
  target_link_libraries(idcs_ctrl_man
    ${catkin_LIBRARIES}
    idcs_hw_vel_ctrl
  )
  
  add_executable(vel_ctrl_man src/vel_ctrl_man.cpp)
  
  target_link_libraries(vel_ctrl_man
    ${catkin_LIBRARIES}
    idcs_hw_vel_ctrl
  )
  
  add_executable(test_rt_prio src/test_rt_prio.cpp)
  
  target_link_libraries(test_rt_prio
    ${catkin_LIBRARIES}
  )

  if( XENOMAI_LIBRARY MATCHES "XENOMAI_LIBRARY-NOTFOUND" )
    message( "Xenomai not found" )
  
    ## Mark executables and/or libraries for installation
    install(TARGETS idcs_hw_vel_ctrl vel_ctrl_man idcs_ctrl_man
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

  else()
    message( "Xenomai found, also installing xeno realtime binaries" )
  
    include_directories(
      include
      /opt/etherlab/include
      ${catkin_INCLUDE_DIRS}
      /usr/xenomai/include
    )
    link_directories(/usr/xenomai/lib)
    
    add_executable(vel_ctrl_man_xeno src/vel_ctrl_man.cpp)
    set_target_properties(vel_ctrl_man_xeno PROPERTIES COMPILE_DEFINITIONS "XENOMAI_REALTIME")
    target_link_libraries(vel_ctrl_man_xeno
      ${catkin_LIBRARIES}
      idcs_hw_vel_ctrl
      xenomai
      native
      rtdm
    )
  
    ## Mark executables and/or libraries for installation
    install(TARGETS idcs_hw_vel_ctrl vel_ctrl_man idcs_ctrl_man vel_ctrl_man_xeno
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

  endif()

else()

  catkin_package(
    INCLUDE_DIRS include
  #  CATKIN_DEPENDS controller_interface hardware_interface roscpp
  )
  message( "Ethercat master libary not found so indradrive controller will not be built." )

endif()

