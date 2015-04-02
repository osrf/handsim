find_package(Boost 1.54.0 COMPONENTS system REQUIRED)

include_directories (
  ${Boost_INCLUDE_DIRS}
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}

  ${GAZEBO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${CCD_LIBRARY_DIRS}
  ${ogre_library_dirs}
  ${ODE_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${Gazebo_LIBRARY_DIRS}
)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")
# Build static ServerFixture library
#set(ServerFixtureSources
  #${PROJECT_SOURCE_DIR}/gazebo/Server.cc
  #${PROJECT_SOURCE_DIR}/gazebo/Master.cc
  #${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc
#  ${PROJECT_SOURCE_DIR}/test/ServerFixture.cc
#)
#add_library(server_fixture STATIC ${ServerFixtureSources})
#add_dependencies(server_fixture gazebo_msgs)
#set(SERVER_FIXTURE_LIBRARY "${PROJECT_BINARY_DIR}/test/libserver_fixture.a")


execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
