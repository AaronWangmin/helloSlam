cmake_minimum_required(VERSION 3.5)
project(pose_graph)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "usr/local/include/g2o" "usr/local/bin/g2o" "usr/local/lib/libg2o_core")

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
    g2o_core g2o_types_sba g2o_solver_cholmod g2o_stuff g2o_types_slam3d 
    ${CHOLMOD_LIBRARIES}
    ${Sophus_LIBRARIES})
