cmake_minimum_required(VERSION 3.5)

project(useLKFlow)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

add_executable(useLKFlow useLKFlow.cpp)
target_link_libraries(useLKFlow ${OpenCV_LIBS})