cmake_minimum_required(VERSION 3.17)
project(slambook2)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
include_directories(ch2)
include_directories(ch3)


#ch2
#add_executable(
#        ch2 ch2/useHello.cpp  ch2/libtest.cpp
#       )

#ch3
add_executable(
       # ch3 ch3/useEigen/eigenMatrix.cpp
        ch3 ch3/useGeometry/useGeometry.cpp)