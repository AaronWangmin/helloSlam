cmake_minimum_required(VERSION 3.5)

project(direct_sparse)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "usr/local/include/g2o" "usr/local/bin/g2o")


add_executable(direct_sparse direct_sparse.cpp)
target_link_libraries(direct_sparse ${OpenCV_LIBS})