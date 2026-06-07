cmake_minimum_required(VERSION 3.20)
project(ballistics_3 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics_3 SHARED src/MissionProcessor.cpp
                                src/providers/JSONTargetProvider.cpp
                                src/config/JSONConfigLoader.cpp
                                src/config/ComponentsFactory.cpp
                                src/solvers/AnalyticalSolver.cpp)
target_include_directories(ballistics_3 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include/interfaces ${CMAKE_CURRENT_LIST_DIR}/include/providers ${CMAKE_CURRENT_LIST_DIR}/include/config ${CMAKE_CURRENT_LIST_DIR}/include/solvers)

add_executable(ballistics_3_cli src/main.cpp)
target_link_libraries(ballistics_3_cli PRIVATE  MissionProcessor
                                                Structs
                                                interfaces/ITargetProvider
                                                interfaces/IConfigLoader
                                                interfaces/IBallisticSolver
                                                providers/JSONTargetProvider
                                                config/JSONConfigLoader
                                                config/ComponentsFactory
                                                solvers/AnalyticalSolver)

target_compile_options(ballistics_3_cli PRIVATE -Wall -Wextra -pedantic)
