cmake_minimum_required(VERSION 2.8.3)
project(haptix_gazebo_plugins)

# for catkin_make
find_package(catkin) # REQUIRED COMPONENTS nothing)
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES HaptixControlPlugin
)

# customize for haptix control plugin
find_package(gazebo REQUIRED)
find_package(polhemus_driver REQUIRED)
find_package(haptix-comm REQUIRED)
find_package(ignition-transport REQUIRED)

# make below target specific?
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${polhemus_driver_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
  ${polhemus_driver_LIBRARY_DIRS}
)

# build library
add_library(HaptixControlPlugin SHARED src/HaptixControlPlugin.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxGrasp.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxSensor.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxDevice.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxCommand.pb.cc)

# additional library dependencies
add_dependencies(HaptixControlPlugin
  gazebo_msgs
)

# library dependencies
target_link_libraries(HaptixControlPlugin
  ${GAZEBO_LIBRARIES}
  ${polhemus_driver_LIBRARY_DIRS}
  polhemus_driver
  ${IGNITION-TRANSPORT_LIBRARIES})

# below adds -std=c++11, needed to compile against ign-transport
set_target_properties(HaptixControlPlugin PROPERTIES
  COMPILE_FLAGS ${IGNITION-TRANSPORT_CXX_FLAGS})

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

# install library and header
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (LIB_INSTALL_DIR_ARCH_INDEPENDENT "lib/")
install (TARGETS HaptixControlPlugin DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

# install includes
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
)

# install worlds
install(DIRECTORY worlds
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

# setup config file
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/setup.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
