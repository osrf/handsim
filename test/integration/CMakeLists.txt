set(TEST_TYPE "INTEGRATION")

set(tests
  hand_grasping.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${HAPTIX-COMM_LIBRARY_DIRS}
)

add_library(test_library ${tests})
target_link_libraries(test_library ${BOOST_LIBRARIES})

# set boost pkgconfig cflags
set (Boost_PKGCONFIG_CFLAGS ${Boost_INCLUDE_DIRS})
if (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
  set (Boost_PKGCONFIG_CFLAGS "-I${Boost_PKGCONFIG_CFLAGS}")
endif (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
string (REPLACE ";" " -I" Boost_PKGCONFIG_CFLAGS "${Boost_PKGCONFIG_CFLAGS}")

# set boost pkgconfig libs
set (Boost_PKGCONFIG_LIBS ${Boost_LIBRARY_DIRS})
if (NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")
  set (Boost_PKGCONFIG_LIBS "-L${Boost_PKGCONFIG_LIBS}")
endif(NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")

foreach (b ${Boost_LIBRARIES})
  get_filename_component(bname ${b} NAME_WE)
  # Prefix always -l
  set (bname "-l${bname}")
  # Remove the prefix lib (not always present, like in pthread)
  string (REPLACE "lib" "" bname "${bname}")
  set (Boost_PKGCONFIG_LIBS "${Boost_PKGCONFIG_LIBS} ${bname}")
endforeach(b)

set_target_properties(test_library PROPERTIES COMPILE_FLAGS ${Boost_PKGCONFIG_CFLAGS})

handsim_build_tests(${tests})
