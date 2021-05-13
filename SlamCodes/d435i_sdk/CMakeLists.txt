cmake_minimum_required(VERSION 3.1.0)
project(d435i_sdk)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

message(STATUS "realsense2_INCLUDE_DIR = ${realsense2_INCLUDE_DIR}")

include_directories(include ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR})

add_library(d435i src/d435i.cpp)
target_link_libraries(d435i ${realsense2_LIBRARY} ${OpenCV_LIBS})

add_executable(test_infrared app/test_infrared.cpp)
target_link_libraries(test_infrared d435i)

