set(TEST_TYPE "REGRESSION")

set(tests
  86_motor_values.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
