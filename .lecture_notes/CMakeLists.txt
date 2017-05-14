project(unscented_kalman_filter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(SOURCE_FILES
        main.cpp
        ukf.cpp
        ukf.h)

add_executable(unscented_kalman_filter ${SOURCE_FILES})