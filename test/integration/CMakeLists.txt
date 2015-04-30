include_directories (
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

link_directories(
  ${HAPTIX-COMM_LIBRARY_DIRS}
)

set(TEST_TYPE "INTEGRATION")

set(tests
  sim_api.cc
  sim_api_client.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
