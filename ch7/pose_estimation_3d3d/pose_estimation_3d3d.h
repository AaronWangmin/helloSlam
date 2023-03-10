#if !defined(_POSE_ESTIMATION_3D3D_)
#define _POSE_ESTIMATION_3D3D

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/sampler.h>
#include <g2o/types/icp/types_icp.h>

using namespace std;


class PoseEstimation3D3D
{
private:
    cv::Mat img_1;
    cv::Mat img_2;
    cv::Mat depth_1;
    cv::Mat depth_2;

    std::vector<cv::KeyPoint> keypoints_1;
    std::vector<cv::KeyPoint> keypoints_2;
    std::vector<cv::DMatch> matches;

    std::vector<cv::Point3f> pts1;
    std::vector<cv::Point3f> pts2;

    cv::Mat R;
    cv::Mat t;

    cv::Mat K = (cv::Mat_<double>(3,3) << 
        520.9, 0, 325,1,
        0, 521.0, 249.7,
        0, 0, 1);

public:
    PoseEstimation3D3D(
        const string &imgFileName_1,
        const string &imgFileName_2,
        const string &depthFileName_1,
        const string &depthFileName_2);

    void find_feature_matches(
        // const cv::Mat &img_1,
        // const cv::Mat &img_2,
        // std::vector<cv::KeyPoint> &keypoints_1,
        // std::vector<cv::KeyPoint> &keypoints_2,
        // std::vector<cv::DMatch> &matches
        );
    void pose_estimation_3d3d(
        // const std::vector<cv::Point3f> &pts1,
        // const std::vector<cv::Point3f> &pts2,
        // cv::Mat &R,
        // cv::Mat &t
    );

    void bundleAdjunstment(
        // const std::vector<cv::Point3f> &pts1,
        // const std::vector<cv::Point3f> &pts2,
        // cv::Mat &R,
        // cv::Mat &t
    );

    cv::Point2d pixel2cam(
        const cv::Point2d &p,
        const cv::Mat &K
    );

    void build3DPoints();

    // ~pose_estimation_3d3d() {}
};

#endif // MACRO



