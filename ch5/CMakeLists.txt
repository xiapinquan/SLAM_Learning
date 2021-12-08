cmake_minimum_required(VERSION 3.17)
project(ch5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")


include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics/imgaeBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBRARIES})

add_executable(undistortImage imageBasics/undistortImage.cpp)
target_link_libraries(undistortImage ${OpenCV_LIBRARIES})

add_executable(stereoVision stereoVision/stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(jointMap rgbd/jointMap.cpp)
target_link_libraries(jointMap ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})