if(EXISTS "/home/python/cpp-miltech/homework_06/ballistics_tests")
  if(NOT EXISTS "/home/python/cpp-miltech/homework_06/ballistics_tests[1]_tests.cmake" OR
     NOT "/home/python/cpp-miltech/homework_06/ballistics_tests[1]_tests.cmake" IS_NEWER_THAN "/home/python/cpp-miltech/homework_06/ballistics_tests" OR
     NOT "/home/python/cpp-miltech/homework_06/ballistics_tests[1]_tests.cmake" IS_NEWER_THAN "${CMAKE_CURRENT_LIST_FILE}")
    include("/usr/share/cmake-3.28/Modules/GoogleTestAddTests.cmake")
    gtest_discover_tests_impl(
      TEST_EXECUTABLE [==[/home/python/cpp-miltech/homework_06/ballistics_tests]==]
      TEST_EXECUTOR [==[]==]
      TEST_WORKING_DIR [==[/home/python/cpp-miltech/homework_06]==]
      TEST_EXTRA_ARGS [==[]==]
      TEST_PROPERTIES [==[]==]
      TEST_PREFIX [==[]==]
      TEST_SUFFIX [==[]==]
      TEST_FILTER [==[]==]
      NO_PRETTY_TYPES [==[FALSE]==]
      NO_PRETTY_VALUES [==[FALSE]==]
      TEST_LIST [==[ballistics_tests_TESTS]==]
      CTEST_FILE [==[/home/python/cpp-miltech/homework_06/ballistics_tests[1]_tests.cmake]==]
      TEST_DISCOVERY_TIMEOUT [==[5]==]
      TEST_XML_OUTPUT_DIR [==[]==]
    )
  endif()
  include("/home/python/cpp-miltech/homework_06/ballistics_tests[1]_tests.cmake")
else()
  add_test(ballistics_tests_NOT_BUILT ballistics_tests_NOT_BUILT)
endif()
