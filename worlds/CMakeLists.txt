set (files 
  arat.world
  arat_left.world
  demo.world
  wood_stack.world
  handsim_empty.world
  luke_hand.world
  luke_left_hand.world
  primitive_hand.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/worlds)
