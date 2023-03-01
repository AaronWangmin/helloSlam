cmake_minimum_required(VERSION 2.8)

project(gaussNewton)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

include_directories(/usr/include/eigen3)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})
# link_directories(${CERES_LIBRARY_DIRS})
# include_directories(/usr/local/include/ceres)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "usr/local/include/g2o" "usr/local/bin/g2o")

find_package(glog 0.5.0 REQUIRED)
find_package(Gflags REQUIRED)


add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} Ceres::ceres)

add_executable(helloCeres helloCeres.cpp)
target_link_libraries(helloCeres Ceres::ceres)

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o_core g2o_stuff glog::glog)
