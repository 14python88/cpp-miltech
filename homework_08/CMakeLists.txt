cmake_minimum_required(VERSION 3.20)
project(ballistics_3 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics_3 STATIC
    src/MissionProcessor.cpp 
    src/providers/JSONTargetProvider.cpp 
    src/config/JSONConfigLoader.cpp 
    src/config/ComponentsFactory.cpp 
    src/solvers/AnalyticalSolver.cpp 
)

target_include_directories(ballistics_3 PUBLIC include)

add_executable(ballistics_3_cli src/main.cpp)
target_link_libraries(ballistics_3_cli PRIVATE ballistics_3)

target_compile_options(ballistics_3_cli PRIVATE -Wall -Wextra -pedantic)
target_compile_options(ballistics_3 PRIVATE -Wall -Wextra -pedantic)
