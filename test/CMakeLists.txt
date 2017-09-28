set(TEST_SOURCES test_main.cpp
        test_save_load_map.cpp)
add_executable(tests ${TEST_SOURCES})
target_link_libraries(tests  Catch  ORB_SLAM2)