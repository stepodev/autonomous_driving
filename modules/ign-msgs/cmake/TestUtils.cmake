#################################################
macro (ign_build_tests)
  # Find the Python interpreter for running the 
  # check_test_ran.py script 
  find_package(PythonInterp QUIET)

  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})

    add_dependencies(${BINARY_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}
      gtest gtest_main
      )

    if (UNIX)
      target_link_libraries(${BINARY_NAME}
         libgtest_main.a
         libgtest.a
         pthread
         ${PROJECT_LIBRARY_TARGET_NAME})
    elseif(WIN32)
      target_link_libraries(${BINARY_NAME}
         gtest.lib
         gtest_main.lib
         ${PROJECT_LIBRARY_TARGET_NAME})

      # Copy in ignition-msgs library
      add_custom_command(TARGET ${BINARY_NAME}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:${PROJECT_NAME}>"
        $<TARGET_FILE_DIR:${BINARY_NAME}> VERBATIM)

    else()
       message(FATAL_ERROR "Unsupported platform")
    endif()

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    if(PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if it didn't.
      # Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
        ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
    endif()
  endforeach()
endmacro()
