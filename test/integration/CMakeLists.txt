set(TEST_TYPE "INTEGRATION")

include(FindBoost)
find_package(Boost REQUIRED COMPONENTS thread system filesystem REQUIRED)

include_directories(
  ${PROJECT_BINARY_DIR}
  ${Boost_FILESYSTEM_INCLUDE_DIRS}
  ${Boost_SYSTEM_INCLUDE_DIRS}
  ${Boost_THREAD_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${Boost_FILESYSTEM_LIBRARY_DIRS}
  ${Boost_SYSTEM_LIBRARY_DIRS}
  ${Boost_THREAD_LIBRARY_DIRS}
  # enable below if gazebo issue 1568 is fixed
  # ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)
set(tests
  physics
)

handsim_build_tests(${tests})
