cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (UNIX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}} -std=c++11")
endif()

include_directories(${PROJECT_SOURCE_DIR}/../include)

# Compiler-specific C++11 activation.
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 4.7))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.8 or greater.")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    if (NOT MSVC12)
        message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 os greater.")
    endif()
else ()
    message(FATAL_ERROR "Your C++ compiler does not support C++11.")
endif ()

# Find the NPTrackingTools library
# ToDo.

include_directories($ENV{NPTRACKINGTOOLS_INC})
link_directories($ENV{NPTRACKINGTOOLS_LIB})

add_executable(optitrackBridge optitrackBridge.cc ../src/OptitrackBridge.cc)
if(WIN32)
  target_link_libraries(optitrackBridge NPTrackingToolsx64 ws2_32)
endif()
