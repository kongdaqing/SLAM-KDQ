cmake_minimum_required(VERSION 2.8.3)
project(viz_scene)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories("/usr/local/include/eigen3/")
message(STATUS  ${OpenCV_INCLUDE_DIRS})
include_directories(inc ${OpenCV_INCLUDE_DIRS})
add_library(libvizscene STATIC src/viz_scene/viz_scene.cpp)
target_link_libraries(libvizscene ${OpenCV_LIBS} -lpthread -lm)
add_executable(test src/main.cpp)
target_link_libraries(test libvizscene)



