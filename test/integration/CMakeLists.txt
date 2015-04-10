include_directories (
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)


set(TEST_TYPE "INTEGRATION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
