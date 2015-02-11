cmake_minimum_required(VERSION 2.8)

# Find the Handsim libraries
find_package(handsim QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${HANDSIM_CXX_FLAGS}")

include_directories(${HANDSIM_INCLUDE_DIRS})
link_directories(${HANDSIM_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR})

add_executable(polhemus_example polhemus_example.c)
target_link_libraries(polhemus_example polhemus_driver)

add_executable(polhemus_example_continuous polhemus_example_continuous.c)
target_link_libraries(polhemus_example_continuous polhemus_driver)

add_executable(polhemus_example_multiple_sensors polhemus_example_multiple_sensors.c)
target_link_libraries(polhemus_example_multiple_sensors polhemus_driver)

add_executable(optitrackBridge optitrackBridge.cc)
target_link_libraries(optitrackBridge HaptixTracking)
