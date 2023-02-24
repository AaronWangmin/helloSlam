cmake_minimum_required(VERSION 2.8)

project(gaussNewton)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(/usr/include/eigen3)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

include_directories(/usr/local/include/ceres)

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} /usr/local/lib/libceres.a)