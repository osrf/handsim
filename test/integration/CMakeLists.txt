include_directories (
  ${HAPTIX-COMM_INCLUDE_DIRS}
)

link_directories(
  ${HAPTIX-COMM_LIBRARY_DIRS}
)

set(TEST_TYPE "INTEGRATION")


set(tests
  sim_api.cc
  sim_api_client.cc
)

find_program(MKOCTFILE mkoctfile)
# Only add the mex-based test if mkoctfile is installed.  This is not a perfect
# check for the presence of Octave and the Octave library bindings, but it
# should cover most cases.
if (MKOCTFILE)
  # The mex test needs to cd into the library installation directory to execute the
  # .m files.
  add_definitions(-DCMAKE_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\"
                  -DCMAKE_INSTALL_LIBDIR=\"${CMAKE_INSTALL_LIBDIR}\")
  set(tests ${tests} sim_api_mex_client.cc)
endif()

link_directories(${PROJECT_BINARY_DIR}/test)

handsim_build_tests(${tests})
