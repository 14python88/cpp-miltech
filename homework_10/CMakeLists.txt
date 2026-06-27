cmake_minimum_required(VERSION 3.20)
project(ballistics_5 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics_5 STATIC
    src/MissionProcessor.cpp 
    src/providers/ThreadSafeTargetProvider.cpp 
    src/config/JSONConfigLoader.cpp 
    src/config/ComponentFactory.cpp 
    src/solvers/AnalyticalSolver.cpp 
    src/Logger.cpp
    src/DronePhysics.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateDecelerating.cpp
)
target_include_directories(ballistics_5 PUBLIC include include/providers include/config include/solvers include/states)

add_executable(ballistics_5_cli src/main.cpp)
target_link_libraries(ballistics_5_cli PRIVATE ballistics_5)

target_compile_options(ballistics_5_cli PRIVATE -Wall -Wextra -pedantic)
target_compile_options(ballistics_5 PRIVATE -Wall -Wextra -pedantic)
