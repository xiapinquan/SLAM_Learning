cmake_minimum_required(VERSION 3.17)
project(ch4)
set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(useSophus useSophus.cpp)
add_executable(trajectoryError example/trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})

