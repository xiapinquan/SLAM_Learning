cmake_minimum_required(VERSION 3.17)
project(slambook2)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
include_directories(ch2)
include_directories(ch3)


#ch1
#add_executable(
#        #ch1 ch1/test.cpp
#        ch1 ch1/vector_test.cpp)

#ch2
#add_executable(
#        ch2 ch2/useHello.cpp  ch2/libtest.cpp
#        )

#ch3
add_executable(
        #ch3 ch3/useEigen/eigenMatrix.cpp
        #ch3 ch3/useGeometry/useGeometry.cpp
        #ch3 ch3/examples/coordinateTransform.cpp
        ch3 ch3/homework/homework_5_eigen.cpp)