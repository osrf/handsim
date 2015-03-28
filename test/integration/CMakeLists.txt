set(TEST_TYPE "INTEGRATION")

set(tests
  hand_grasping.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
