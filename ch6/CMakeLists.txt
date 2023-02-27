cmake_minimum_required(VERSION 2.8)

project(gaussNewton)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(/usr/include/eigen3)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})
# link_directories(${CERES_LIBRARY_DIRS})
# include_directories(/usr/local/include/ceres)

find_package(glog 0.5.0 REQUIRED)
find_package(Gflags REQUIRED)


add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} Ceres::ceres)

add_executable(helloCeres helloCeres.cpp)
target_link_libraries(helloCeres Ceres::ceres)
