set (files
  arat.world
  arat_left.world
  handsim_empty.world
  luke_hand.world
  luke_hand_spring_test.world
  primitive_hand.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/worlds)
