set(TEST_TYPE "INTEGRATION")

set(tests
  hand_grasping.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
)

handsim_build_tests(${tests})
