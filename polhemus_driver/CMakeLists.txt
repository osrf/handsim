cmake_minimum_required(VERSION 2.8.3)
project(polhemus_driver)
#set(CMAKE_BUILD_TYPE "Debug")

add_definitions(-Wall -Werror)

find_package(catkin) # REQUIRED COMPONENTS nothing)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polhemus_driver
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include  #hm, is this ok?
)

add_library(polhemus_driver SHARED src/polhemus_driver.c)
target_link_libraries(polhemus_driver usb-1.0)

add_executable(polhemus_example src/example.c)
target_link_libraries(polhemus_example polhemus_driver)
add_executable(polhemus_example_continuous src/example_continuous.c)
target_link_libraries(polhemus_example_continuous polhemus_driver)
add_executable(polhemus_example_multiple_sensors src/example_multiple_sensors.c)
target_link_libraries(polhemus_example_multiple_sensors polhemus_driver)


# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

# install libraries
install(TARGETS polhemus_driver DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

# install includes
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(PKG_NAME ${PROJECT_NAME})
set(cmake_conf_file "cmake/polhemus_driver-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME_LOWER}
        COMPONENT cmake)
