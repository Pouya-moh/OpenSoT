if(WIN32)
    SET(CTEST_CUSTOM_POST_TEST "${MATLAB_EXECUTABLE} -nosplash -nodisplay -r \"cd ${CMAKE_CURRENT_BINARY_DIR}, try, run ('plot_test_results.m'), end, quit\"; python ${CMAKE_CURRENT_BINARY_DIR}/plot_test_results.py")
else()
    SET(CTEST_CUSTOM_POST_TEST "${MATLAB_EXECUTABLE} -nosplash -nodisplay -r \"cd ${CMAKE_CURRENT_BINARY_DIR}, try, run ('plot_test_results.m'), end, quit\"; python ${CMAKE_CURRENT_BINARY_DIR}/plot_test_results.py")
endif()