cmake_minimum_required(VERSION 3.20)
project(ballistics_2 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# add_library(ballistics SHARED src/ballistics.cpp)
# target_include_directories(ballistics PUBLIC include)

add_executable(ballistics_2_cli src/main.cpp)
# target_link_libraries(ballistics_2_cli PRIVATE ballistics)

target_compile_options(ballistics_2_cli PRIVATE -Wall -Wextra -pedantic)
