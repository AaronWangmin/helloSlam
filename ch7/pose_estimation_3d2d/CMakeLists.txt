cmake_minimum_required(VERSION 3.5)
project(pose_estimation_3d2d)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
# set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories("/usr/include/eigen3")

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR} "usr/local/include/g2o" "usr/local/bin/g2o")

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
# target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} 
#     g2o_types_sba g2o_csparse_extension ${Sophus_LIBRARIES}
#     g2o_core g2o_solver_eigen)

target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} ${Sophus_LIBRARIES}
    g2o_core g2o_types_sba g2o_solver_eigen g2o_stuff g2o_types_slam3d)
 