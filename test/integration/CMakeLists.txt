include_directories (
  ${Boost_FILESYSTEM_INCLUDE_DIRS}
  ${Boost_SYSTEM_INCLUDE_DIRS}
  ${Boost_THREAD_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

link_directories(
  ${Boost_FILESYSTEM_LIBRARY_DIRS}
  ${Boost_SYSTEM_LIBRARY_DIRS}
  ${Boost_THREAD_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
)


set(TEST_TYPE "INTEGRATION")

set(tests
  physics.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
