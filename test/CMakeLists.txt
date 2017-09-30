set(TEST_SOURCES test_main.cpp
        )
add_executable(tests ${TEST_SOURCES})

target_link_libraries(tests
        Catch
        ORB_SLAM2
        model_based_localization)


add_executable(test_save_load_map  test_save_load_map.cpp)
target_link_libraries(test_save_load_map
        ORB_SLAM2
        model_based_localization)
