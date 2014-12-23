set(TEST_TYPE "REGRESSION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
