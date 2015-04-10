include_directories (
  #${ODE_INCLUDE_DIRS}
  #${OPENGL_INCLUDE_DIR}
  #${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  #${PROTOBUF_INCLUDE_DIR}
)

link_directories(
  #${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  #${ODE_LIBRARY_DIRS}
)


set(TEST_TYPE "INTEGRATION")

set(tests
  hand_grasping.cc
)

#target_link_libraries(tests ${Boost_LIBRARIES})

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
