cmake_minimum_required(VERSION 2.8.3)
project(viz_scene)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS  ${OpenCV_INCLUDE_DIRS})
message(STATUS ${EIGEN3_INCLUDE_DIR})
include_directories(inc ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_library(libvizscene STATIC src/viz_scene/viz_scene.cpp
            src/imu_motion/imu_motion.cpp)
target_link_libraries(libvizscene ${OpenCV_LIBS} -lpthread -lm)
add_executable(testCameraMotion src/testCameraMotion.cpp)
target_link_libraries(testCameraMotion libvizscene)



