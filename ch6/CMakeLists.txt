cmake_minimum_required(VERSION 3.17)
project(ch6)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")

#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(gaussNeuton gaussNeuton.cpp)
target_link_libraries(gaussNeuton ${OpenCV_LIBRARIES})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

#message(FATAL_ERROR  "xpq ===  ${G2O_CORE_LIBRARY}")
