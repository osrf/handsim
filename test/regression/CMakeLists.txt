set(TEST_TYPE "REGRESSION")

set(tests
  86_motor_values.cc
)

include_directories (
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${HAPTIX-COMM_LIBRARY_DIRS}
)

handsim_build_tests(${tests})
