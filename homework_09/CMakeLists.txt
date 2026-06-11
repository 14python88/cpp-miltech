cmake_minimum_required(VERSION 3.20)
project(ballistics_4 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics_4 STATIC
    src/MissionProcessor.cpp 
    src/providers/JSONTargetProvider.cpp 
    src/config/JSONConfigLoader.cpp 
    src/config/ComponentFactory.cpp 
    src/solvers/AnalyticalSolver.cpp 
    src/Logger.cpp
)
target_include_directories(ballistics_4 PUBLIC include include/providers include/config include/solvers)

add_executable(ballistics_4_cli src/main.cpp)
target_link_libraries(ballistics_4_cli PRIVATE ballistics_4)

target_compile_options(ballistics_4_cli PRIVATE -Wall -Wextra -pedantic)
target_compile_options(ballistics_4 PRIVATE -Wall -Wextra -pedantic)
