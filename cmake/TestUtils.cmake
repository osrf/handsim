#################################################
macro (handsim_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)

    include_directories(
      ${PROJECT_BINARY_DIR}
      ${Boost_FILESYSTEM_INCLUDE_DIRS}
      ${Boost_SYSTEM_INCLUDE_DIRS}
      ${GAZEBO_INCLUDE_DIRS}
      ${HAPTIX-COMM_INCLUDE_DIRS}
      ${IGNITION-TRANSPORT_INCLUDE_DIRS}
    )
    link_directories(
      ${PROJECT_BINARY_DIR}/test
      ${Boost_FILESYSTEM_LIBRARY_DIRS}
      ${Boost_SYSTEM_LIBRARY_DIRS}
      ${GAZEBO_LIBRARY_DIRS}
      ${HAPTIX-COMM_LIBRARY_DIRS}
      ${IGNITION-TRANSPORT_LIBRARY_DIRS}
    )

    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${PROJECT_SOURCE_DIR}/test/ServerFixture.cc)

    add_dependencies(${BINARY_NAME}
      lib${PROJECT_NAME_LOWER}
      gtest gtest_main
      )

    target_link_libraries(${BINARY_NAME}
      libgtest.a
      libgtest_main.a
      pthread
      HaptixTracking
      ${Boost_FILESYSTEM_LIBRARIES}
      ${Boost_SYSTEM_LIBRARIES}
      boost_system
      boost_thread
      boost_filesystem
      ${IGNITION-TRANSPORT_LIBRARIES}
      ${HAPTIX-COMM_LIBRARIES}
      ${GAZEBO_LIBRARIES}
      )

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  endforeach()
endmacro()
