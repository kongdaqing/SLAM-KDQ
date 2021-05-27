if(DEFINED ENV{ROS_ENABLE})

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
project(vio_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
    cv_bridge
    rosbag
)
find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
  include/vio
  ${OPENCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_library(volib src/FeatureManager.cpp
                  src/Initializator.cpp
                  src/Estimator.cpp
                  src/FeatureTracker.cpp)
target_link_libraries(volib ${OpenCV_LIBS})    

add_executable(test_camera_node app/test_camera_node.cpp)
target_link_libraries(test_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  volib
)

add_executable(test_feature_track app/test_featureTrack_node.cpp)
target_link_libraries(test_feature_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  volib
)

else()



cmake_minimum_required(VERSION 3.0.1)
project(vio)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
message(STATUS ${PARENT_DIRECTORY})
set(D435I_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/d435i")
set(D435I_LIBS "${PARENT_DIRECTORY}/env/lib/d435i/libd435i.so")
set(REPLAYKIT_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/replaykit")
set(REPLAYKIT_LIBS "${PARENT_DIRECTORY}/env/lib/replaykit/libreplaykit.a;${PARENT_DIRECTORY}/env/lib/replaykit/libreplay_proto.a")

if (APPLE)
find_package(OpenCV 4.0 REQUIRED)
else()
set(OpenCV_DIR "/home/kdq/packages/opencv4/opencv/build")
find_package(OpenCV 4.0 REQUIRED)
#message(STATUS "OpenCV_Path = ${OpenCV_INCLUDE_DIRS}")
endif()
find_package(realsense2 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kindr REQUIRED)

include_directories(
  include/vio
  include/nnstation
  include/utility
  ${OpenCV_INCLUDE_DIRS}
  ${D435I_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${REPLAYKIT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)
add_library(volib src/FeatureManager.cpp
                  src/Initializator.cpp
                  src/Estimator.cpp
                  src/FeatureTracker.cpp
                  src/Simulator.cpp
                  src/VizScene.cpp)
target_link_libraries(volib ${OpenCV_LIBS} -lpthread)     

add_executable(vo app/main.cpp)
target_link_libraries(vo volib)

add_executable(test_camera app/test_camera.cpp)
target_link_libraries(test_camera volib)

add_executable(test_camera2 app/test_camera2.cpp)
target_link_libraries(test_camera2  volib)

add_executable(test_initializator app/test_initializator.cpp)
target_link_libraries(test_initializator volib)

add_executable(test_vizScene app/test_vizScene.cpp)
target_link_libraries(test_vizScene volib)


add_executable(test_pnpSolver app/test_pnpSolver.cpp)
target_link_libraries(test_pnpSolver volib)


add_executable(test_estimator app/test_estimator.cpp)
target_link_libraries(test_estimator ${OpenCV_LIBS} volib)

add_executable(test_d435i_feature_track app/test_d435i_feature_track.cpp)
target_link_libraries(test_d435i_feature_track  ${D435I_LIBS}  volib)

add_executable(test_replaykit_online app/test_replaykit_online.cpp)
target_link_libraries(test_replaykit_online   ${PROTOBUF_LIBRARIES} nanomsg ${realsense2_LIBRARY} ${REPLAYKIT_LIBS} volib)

add_executable(d435i_estimator app/d435i_estimator.cpp)
target_link_libraries(d435i_estimator ${realsense2_LIBRARY} ${D435I_LIBS}  volib)



endif()