if(WIN32)
    SET(CTEST_CUSTOM_POST_TEST "${MATLAB_EXECUTABLE} -nosplash -nodisplay -r \"cd ${CMAKE_CURRENT_BINARY_DIR}\tests; try, run ('plot_test_results.m'); end; quit\"")
else()
    SET(CTEST_CUSTOM_POST_TEST "${MATLAB_EXECUTABLE} -nosplash -nodisplay -r \"cd ${CMAKE_CURRENT_BINARY_DIR}/tests; try, run ('plot_test_results.m'); end; quit\"")
endif()