cmake_minimum_required(VERSION 3.17)
project(ch2)
set(CMAKE_CXX_STANDARD 14)
#include_directories("test.h")
#add_library(lib SHARED libtest.cpp)

add_executable(useHelloLib useHello.cpp)
target_link_libraries(useHelloLib ${PROJECT_SOURCE_DIR}/build/liblib.so)
