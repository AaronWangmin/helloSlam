#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void triangulation(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points
)
{
    Mat T1 = (Mat_<float>(3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float>(3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),t.at<double>(2,0)
        );

    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2f> pts_1,pts_2;
    for (DMatch m : matches)
    {
        pts_1.push_back(pixel2cam(keypoints_1[m.queryIdx].pt,K));
        pts_2.push_back(pixel2cam(keypoints_2[m.trainIdx].pt,K));
    }

    Mat pts_4d;
    triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);

    for (int i = 0; i < pts_4d.cols; i++)
    {
        Mat x = pts_4d.col(i);
        x / = x.at<float>(3,0);
        Point3d p(
           x.at<float>(0,0), 
           x.at<float>(1,0), 
           x.at<float>(2,0), 
        );
        points.push_back(p);
    }
    
}