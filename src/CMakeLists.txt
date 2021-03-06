set (gtest_sources
  OptitrackBridge_TEST.cc
)

# make below target specific?
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)

#################################################
# Build tracking library (Polhemus + Optitrack).
add_library(HaptixTracking SHARED polhemus_driver.c Optitrack.cc
  OptitrackBridge.cc)
target_link_libraries(HaptixTracking usb-1.0
  ${GAZEBO_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES})
install(TARGETS HaptixTracking DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

#################################################
# Build Gazebo plugins

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

# customize for haptix control plugin
QT4_WRAP_CPP(HaptixGUIPlugin_MOC
  ${PROJECT_SOURCE_DIR}/include/handsim/HaptixGUIPlugin.hh
  ${PROJECT_SOURCE_DIR}/include/handsim/TaskButton.hh)
add_library(HaptixGUIPlugin SHARED ${HaptixGUIPlugin_MOC}
            HaptixGUIPlugin.cc TaskButton.cc
            ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxGrasp.pb.cc)

# build library
add_library(HaptixControlPlugin SHARED HaptixControlPlugin.cc
  WrenchHelper.cc
  JointHelper.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxGrasp.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxSensor.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxRobot.pb.cc
  ${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxCommand.pb.cc
)

add_library(HaptixWorldPlugin SHARED HaptixWorldPlugin.cc WrenchHelper.cc)

# library dependencies
target_link_libraries(HaptixControlPlugin
  ${GAZEBO_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  HaptixTracking)

target_link_libraries(HaptixGUIPlugin ${GAZEBO_LIBRARIES}
  ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES}
  haptix-comm HaptixTracking)

target_link_libraries(HaptixWorldPlugin
  ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES} libhaptix-msgs.a haptix-comm
)

# below adds -std=c++11, needed to compile against ign-transport
set_target_properties(HaptixControlPlugin HaptixGUIPlugin HaptixWorldPlugin PROPERTIES
  COMPILE_FLAGS ${IGNITION-TRANSPORT_CXX_FLAGS})

install (TARGETS HaptixControlPlugin HaptixGUIPlugin HaptixWorldPlugin DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

handsim_build_tests(${gtest_sources})
