cmake_minimum_required(VERSION 2.6)
project(data_interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_INSTALL_PREFIX "../../env/")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "../lib")
set(EXECUTABLE_OUTPUT_PATH "../bin")
#构建时候用add_custom_target
add_custom_target(create_lib ALL COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
#配置的时候用file
# file(MAKE_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
# file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX})
find_package(OpenCV REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/replaykit.proto proto/image.proto proto/rovio.proto)
add_library(replay_proto STATIC ${PROTO_SRCS})
target_include_directories(replay_proto PUBLIC ${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(replay_proto ${PROTOBUF_LIBRARIES})


include_directories(
  include
  ${PROTOBUF_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})



add_executable(test 
  src/main.cpp)
target_link_libraries(test 
  ${OpenCV_LIBS}
  replay_proto
  nanomsg)

# INSTALL(TARGETS ZeroZeroImu
#   ARCHIVE DESTINATION lib/data_interface)
# INSTALL(DIRECTORY inc/ DESTINATION include/data_interface) 
