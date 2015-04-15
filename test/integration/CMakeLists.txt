include_directories (
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)

set(TEST_TYPE "INTEGRATION")

set(tests
  sim_api.cc
)

set(qt_tests
  sim_api_gui.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
# TODO this macro doesn't work at the moment
#handsim_build_tests(${qt_tests})
