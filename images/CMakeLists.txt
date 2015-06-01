set (files
     hanoi.png
     pyramid.png
     block_stack.png

     grasp_1.jpg
     grasp_3.jpg
     grasp_2.jpg
     grasp_4.jpg
     grasp_5.jpg
     grasp_6.jpg
     grip_1.jpg
     grip_3.jpg
     grip_2.jpg
     grip_4.jpg
     hand_left.svg
     hand_right.svg
     pinch_1.jpg
     pinch_2.jpg
     settings.png
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/arat/arat_icons)
