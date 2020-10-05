cmake_minimum_required(VERSION 2.6)
project(data_interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_INSTALL_PREFIX "../../env/")


find_package(OpenCV REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  inc
  ${PROTOBUF_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

add_library(ZeroZeroImu STATIC 
  src/zz_vio_interface/rovio.pb.cc)
target_link_libraries(ZeroZeroImu 
  ${PROTOBUF_LIBRARIES})



add_executable(test 
  src/main.cpp)
target_link_libraries(test 
  ${OpenCV_LIBS}
  ZeroZeroImu
  nanomsg)

INSTALL(TARGETS ZeroZeroImu
  ARCHIVE DESTINATION lib/data_interface)
INSTALL(DIRECTORY inc/ DESTINATION include/data_interface) 
