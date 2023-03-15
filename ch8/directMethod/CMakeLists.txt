cmake_minimum_required(VERSION 3.5)

project(direct_sparse)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(glog REQUIRED)


include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "usr/local/include/g2o" "usr/local/bin/g2o")


add_executable(direct_sparse direct_sparse.cpp)
target_link_libraries(direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} 
    g2o_core g2o_types_sba g2o_solver_eigen g2o_stuff g2o_types_slam3d glog::glog)
