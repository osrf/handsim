set (folders
  mpl_haptix_right_forearm
  mpl_haptix_left_forearm
  spring_buckle_test_1
  spring_buckle_test_2
  spring_buckle_test_3
)

foreach (folder ${folders})
  install(DIRECTORY ${folder}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/models)
endforeach()
