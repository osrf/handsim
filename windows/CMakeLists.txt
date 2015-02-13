cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (WIN32)
  # ToDo: Make sure that the library and header are installed.
  include_directories($ENV{NPTRACKINGTOOLS_INC})
  link_directories($ENV{NPTRACKINGTOOLS_LIB})

  add_executable(optitrackBridge optitrackBridge.cc ../src/OptitrackBridge.cc)
  target_link_libraries(optitrackBridge NPTrackingToolsx64 ws2_32)
else ()
  message(FATAL_ERROR "This program is only supported on Windows.")
endif ()
