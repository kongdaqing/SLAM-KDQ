cmake_minimum_required(VERSION 2.8.3)
project(vo_initialization)
find_package(OpenCV REQUIRED)
set(Eigen3_DIR "/usr/include/eigen3")
include_directories(inc ${Eigen3_DIR}
${OpenCV_INCLUDE_DIRS})
message(STATUS ${OpenCV_INCLUDE_DIRS})
add_executable(vo_ini src/main.cpp)
target_link_libraries(vo_ini ${OpenCV_LIBS})