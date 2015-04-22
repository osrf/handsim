link_directories(
  ${CCD_LIBRARY_DIRS}
  #${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  #${ODE_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories (
  #${ODE_INCLUDE_DIRS}
  #${OPENGL_INCLUDE_DIR}
  #${OGRE_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test

  ${PROJECT_BINARY_DIR}/test # need this for test_config.h
)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test/test_config.h)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

# Build static ServerFixture library
include_directories(
  ${PROJECT_BINARY_DIR}
  ${Boost_FILESYSTEM_INCLUDE_DIRS}
  ${Boost_SYSTEM_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${Boost_FILESYSTEM_LIBRARY_DIRS}
  ${Boost_SYSTEM_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)
set(ServerFixtureSources
  ServerFixture.cc
)
# Build the server_fixture static library
add_library(server_fixture STATIC ${ServerFixtureSources})

add_subdirectory(integration)
# add_subdirectory(performance)
# add_subdirectory(regression)
