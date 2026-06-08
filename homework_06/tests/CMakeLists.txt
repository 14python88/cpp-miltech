include(GoogleTest)
 
add_executable(ballistics_tests
    ballistics_tests
)
 
target_link_libraries(ballistics_tests
    PRIVATE
        ballistics
        GTest::gtest_main
)
 
# Expose the source-tree data directory to the test binary at compile time.
# Tests use TEST_DATA_DIR"/zero_zd.txt" — no runtime path logic needed.
target_compile_definitions(ballistics_tests
    PRIVATE
        TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)
 
gtest_discover_tests(ballistics_tests)