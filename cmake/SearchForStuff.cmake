include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

find_package(gazebo REQUIRED)
find_package(haptix-comm REQUIRED)
find_package(ignition-transport REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system filesystem thread REQUIRED)

# for haptix GUI plugin
find_package(Qt4)
