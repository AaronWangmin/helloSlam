#ifndef _POSE_ESTIMATION_3D2D
#define _POSE_ESTIMATION_3D2D

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat &img_1, 
    const Mat &img_2,
    vector<KeyPoint> &keypoints_1,
    vector<KeyPoint> &keypoints_2,
    vector<DMatch> &matches);

// pixel coordinate transfer to cameral coordinate
Point2d pixel2cam(
    const Point2d &p, 
    const Mat &K);

void bundleAdjustment(
    const vector<Point3f> points_3d,
    const vector<Point2f> poiints_2d,
    const Mat &K,
    Mat &R,
    Mat &t);


#endif


