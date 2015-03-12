set (files 
  arat.world
  arat_left.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds)
