set (files 
  arat.world
  arat_left.world
  wood_stack.world
  handsim_empty.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds)
