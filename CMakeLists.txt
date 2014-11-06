cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (PROJECT_NAME "handsim")

string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set (PROJECT_MAJOR_VERSION 0)
set (PROJECT_MINOR_VERSION 2)
set (PROJECT_PATCH_VERSION 0)

set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION_FULL}")

message (STATUS "===========================================================")
message (STATUS "This CMakeLists.txt on top level is only for generating the")
message (STATUS "source package. To compile and run the code please read: ")
message (STATUS "README.md")
message (STATUS "===========================================================")

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")

########################################
# Package Creation: 
include (${project_cmake_dir}/cpack.cmake)
set (CPACK_PACKAGE_VERSION "${PROJECT_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${PROJECT_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${PROJECT_PATCH_VERSION}")

if (CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

  configure_file("${project_cmake_dir}/cpack_options.cmake.in"
    ${PROJECT_CPACK_CFG_FILE} @ONLY)

  set(CPACK_PROJECT_CONFIG_FILE ${PROJECT_CPACK_CFG_FILE})
  include (CPack)
endif()

# If we're configuring only to package source, stop here
if (PACKAGE_SOURCE_ONLY)
  message(WARNING "Configuration was done in PACKAGE_SOURCE_ONLY mode."
  "You can build a tarball (make package_source), but nothing else.")
  return()
endif()
