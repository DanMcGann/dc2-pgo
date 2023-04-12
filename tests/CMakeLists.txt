file(GLOB test_executables "test*.cpp")

foreach(test ${test_executables})
    get_filename_component(stem ${test} NAME_WE)
    add_executable(${stem} ${test})
    target_link_libraries(${stem} DPGO gtest gtest_main)
    add_test(${stem} ${stem})
endforeach()
