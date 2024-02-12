cmake_minimum_required(VERSION 3.5)
project(testing_task)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)
add_executable(testing_task
        src/main.cpp
        src/PathFinder.cpp
)
