include (${project_cmake_dir}/Utils.cmake)

set (headers
  OptitrackBridge.hh
)

set (handsim_headers "" CACHE INTERNAL "Ignition headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(handsim_headers
    "Handsim headers" "#include <handsim/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/handsim_auto_headers.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/handsim.hh)

handsim_install_includes("" ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hh)
handsim_install_includes(${PROJECT_NAME} ${headers})
