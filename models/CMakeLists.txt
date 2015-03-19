set (folders 
  mpl_haptix_right_forearm
  mpl_haptix_left_forearm
)

foreach (folder ${folders})
  install(DIRECTORY ${folder}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models)
endforeach()
