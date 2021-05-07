if(DEFINED ENV{ROS_ENABLE})

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
project(vo_ros)

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
  include/vo
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



cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
project(vo)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_Path = ${OpenCV_INCLUDE_DIRS}")

include_directories(
  include/vo
  ${OpenCV_INCLUDE_DIRS}
)
add_library(volib src/FeatureManager.cpp
                  src/Initializator.cpp
                  src/Estimator.cpp
                  src/FeatureTracker.cpp
                  src/Simulator.cpp
                  src/VizScene.cpp)
target_link_libraries(volib ${OpenCV_LIBS} -lpthread)     

add_executable(vo app/main.cpp)
target_link_libraries(vo ${OpenCV_LIBS} volib)

add_executable(test_camera app/test_camera.cpp)
target_link_libraries(test_camera ${OpenCV_LIBS} volib)

add_executable(test_camera2 app/test_camera2.cpp)
target_link_libraries(test_camera2 ${OpenCV_LIBS} volib)

add_executable(test_initializator app/test_initializator.cpp)
target_link_libraries(test_initializator ${OpenCV_LIBS} volib)

add_executable(test_vizScene app/test_vizScene.cpp)
target_link_libraries(test_vizScene ${OpenCV_LIBS} volib)


add_executable(test_pnpSolver app/test_pnpSolver.cpp)
target_link_libraries(test_pnpSolver ${OpenCV_LIBS} volib)


add_executable(test_estimator app/test_estimator.cpp)
target_link_libraries(test_estimator ${OpenCV_LIBS} volib)

endif()