cmake_minimum_required(VERSION 3.17)
project(ch3)
set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")

add_executable(eigenMatrix useEigen/eigenMatrix.cpp)
add_executable(useGeometry useGeometry/useGeometry.cpp)
add_executable(coordinateTransform examples/coordinateTransform.cpp)
add_executable(homework homework/homework_5_eigen.cpp)
