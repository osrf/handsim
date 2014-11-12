cmake_minimum_required(VERSION 2.8.3)
project(sr_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES sr_description
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
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

# Look for xacro, and if found, declare a rule to use it to generate the .urdf
# from the .xacro files.
find_package(xacro)
find_program(XACRO_EXE xacro
             PATHS ${xacro_DIR}/../../../lib/xacro)
if(XACRO_EXE)
  # Here we overwriting the urdf file in the source tree, which isn't very
  # clean, but is better than the alternatives that we thought of.
  add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/handsupport_and_hand_motor.urdf
                     COMMAND
		     ROS_PACKAGE_PATH=${CMAKE_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} ${XACRO_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/robots/handsupport_and_hand_motor.urdf.xacro > ${CMAKE_CURRENT_SOURCE_DIR}/handsupport_and_hand_motor.urdf
                     DEPENDS 
		     ${CMAKE_CURRENT_SOURCE_DIR}/robots/handsupport_and_hand_motor.urdf.xacro
		     ${CMAKE_CURRENT_SOURCE_DIR}/hand/xacro/full_hand_motor.urdf.xacro
		     ${CMAKE_CURRENT_SOURCE_DIR}/other/xacro/gazebo/gazebo.urdf.xacro
		     ${CMAKE_CURRENT_SOURCE_DIR}/arm/xacro/hand_support/shadowarm_handsupport_motor.urdf.xacro
		     ${CMAKE_CURRENT_SOURCE_DIR}/materials.urdf.xacro)
  add_custom_target(handsupport_and_hand_motor.urdf_gen ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/handsupport_and_hand_motor.urdf)
else()
  message("You're missing xacro, so you cannot regenerate .urdf files from .urdf.xacro files")
endif()

## Declare a cpp library
# add_library(sr_description
#   src/${PROJECT_NAME}/sr_description.cpp
# )

## Declare a cpp executable
# add_executable(sr_description_node src/sr_description_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sr_description_node sr_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sr_description_node
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
# install(TARGETS sr_description sr_description_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

# Install urdf model file
install(DIRECTORY 
  arm hand other
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo_models/${PROJECT_NAME}
)
install(FILES
  model.config sr_description.urdf materials.urdf.xacro
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo_models/${PROJECT_NAME}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sr_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
