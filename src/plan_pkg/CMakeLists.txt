cmake_minimum_required(VERSION 2.8.3)
project(plan_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
	descartes_core
  descartes_moveit
  descartes_trajectory
	descartes_planner
  nav_msgs
  geometry_msgs
	trajectory_msgs
	moveit_msgs
  shape_msgs
  message_generation
	moveit_ros_move_group
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_robot_interaction
	pluginlib
	trac_ik_lib
	tutorial_utilities REQUIRED
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#moveit_ros_move_group
#moveit_ros_planning


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
##generate_messages(
##  DEPENDENCIES
##  std_msgs  # Or other packages containing msgs
##  shape_msgs
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
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES request
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
	../descartes
	include
)

## Declare a cpp library
# add_library(request
#   src/${PROJECT_NAME}/request.cpp
# )

## Declare a cpp executable
add_executable(sim src/planner.cpp)
add_executable(tSim src/trac_planner.cpp)
add_executable(ikSim 
	src/ikplanner.cpp
	utils/ur_kin.cpp
)
#add_executable(${PROJECT_NAME}_tutorial2 src/tutorial2.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes


## Specify libraries to link a library or executable target againstcatkin
target_link_libraries(sim
  ${catkin_LIBRARIES} 
)
target_link_libraries(tSim
  ${catkin_LIBRARIES} 
)
target_link_libraries(ikSim
  ${catkin_LIBRARIES} 
)
#target_link_libraries(${PROJECT_NAME}_tutorial2
#  ${catkin_LIBRARIES}
#)


#set_target_properties(${PROJECT_NAME}_tutorial2
#  PROPERTIES OUTPUT_NAME tutorial2
#  PREFIX "")

## is this necessary?
#install(TARGETS ${PROJECT_NAME}_tutorial2
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)





