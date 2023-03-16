#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using Sophus::SO3;
using Sophus::SE3;

// cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std
#include <string>
#include <vector>
#include <list>
#include <set>
#include <unordered_map>
#include <map>
#include <memory>
#include <iostream>

using namespace std;

#endif