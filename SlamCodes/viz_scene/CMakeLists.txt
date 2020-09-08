cmake_minimum_required(VERSION 2.8.3)
project(viz_scene)
find_package(OpenCV REQUIRED)
message(STATUS  ${OpenCV_INCLUDE_DIRS})
include_directories(inc ${OpenCV_INCLUDE_DIRS})
add_library(libvizscene STATIC src/viz_scene/viz_scene.cpp)
target_link_libraries(libvizscene ${OpenCV_LIBS})
add_executable(test src/main.cpp)
target_link_libraries(test libvizscene)



