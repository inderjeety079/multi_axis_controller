cmake_minimum_required(VERSION 2.8.3)
project(interface)
set(CMAKE_VERBOSE_MAKEFILE ON)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BOOST_CXX_FLAGS} -std=c++14" )
set (CMAKE_CXX_FLAGS "-g")
find_package(catkin REQUIRED COMPONENTS
#  dynamic_reconfigure
#  geometry_msgs
#  roscpp
#  rosconsole
#  std_msgs
#  tf
#  nav_msgs
#  rosbag
#  genmsg
#  message_generation
#  pluginlib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(jsoncpp REQUIRED)
find_package(spdlog REQUIRED
	HINTS /usr/local/lib/)
#find_package(Boost REQUIRED)
#find_package(Threads REQUIRED)

#add_message_files(
#  FILES
#  IPU.msg
#  encoder.msg
#  heading.msg
#)

#add_service_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  nav_msgs std_msgs geometry_msgs
#  )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )



#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        tf
#        geometry_msgs
#        rosbag
#        nav_msgs
#)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interface
#  CATKIN_DEPENDS roscpp
#  DEPENDS jsoncpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/jsoncpp
  /usr/local/include
  
)

add_library(${PROJECT_NAME} SHARED
			src/config_parser.cpp
            src/udp_connection.cpp
            src/udp_server.cpp
            src/tcp_connection.cpp
            src/tcp_server.cpp
            src/connection_manager.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} jsoncpp)

#add_executable(RTInterfaceNode src/rt_ros_interface_node.cpp)
#target_include_directories(RTInterfaceNode PUBLIC
#        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
#        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDE_DIR}>")
#add_dependencies(RTInterfaceNode rt_interface_generate_messages_cpp)
# spdlog_enable_warnings(RTInterfaceNode)
#target_link_libraries(RTInterfaceNode ${catkin_LIBRARIES} ${PROJECT_NAME} jsoncpp spdlog::spdlog_header_only)

#target_link_libraries(RTInterfaceNode PRIVATE spdlog)

install(TARGETS ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
  #install(TARGETS multi_axis_controller  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ##        EXPORT  ${PROJECT_NAME}  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  #        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  #        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}")



## Declare a C++ library
# add_library(butler_gazebo_simulation
#   src/${PROJECT_NAME}/butler_gazebo_simulation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(butler_gazebo_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(butler_gazebo_simulation_node src/butler_gazebo_simulation_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(butler_gazebo_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(butler_gazebo_simulation_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS butler_gazebo_simulation butler_gazebo_simulation_node
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

#install(TARGETS butler_remote_listener
#       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#       )


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_butler_gazebo_simulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#add_library(Warehouse SHARED plugins/warehouse.cc)
#target_link_libraries(Warehouse
#  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
#)

#add_library(ButlerPlugin SHARED plugins/butler.cc)
#target_link_libraries(ButlerPlugin
#  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
#)
