#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(const cv::Mat &img_1,const cv::Mat &img_2,
    vector<cv::KeyPoint> &keypoints_1,vector<cv::KeyPoint> &keypoints_2,
    vector<cv::DMatch> &matches)
{
    
    Mat descriptors_1,descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);
    matcher->match(descriptors_1,descriptors_2,matches);    

    auto min_max = minmax_element(matches.begin(),matches.end(),
        [] (const DMatch &m1,const DMatch &m2) {return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;    

    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist,30.0))
        {
            good_matches.push_back(matches[i]);
        }
        
    }
    
    matches = good_matches;

}


void pose_estimation_2d2d(std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches,
    cv::Mat &R,Mat &t)
{
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points1,points2,CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    cv::Point2d principal_point(325.1,249.7);
    double focal_length = 521;
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1,points2,focal_length,principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points1,points2,CV_RANSAC,3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;

    cv::recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;

}

int main(int argc,char** argv)
{
    if (argc != 3)
    {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }

    cv::Mat img_1 = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2],CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    cout << "the total number of matched-points is " << matches.size() << endl;

    cv::Mat R,t;
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);

    cv::Mat t_x = (cv::Mat_<double>(3,3) << 0, -t.at<double>(2,0),t.at<double>(1,0),
        t.at<double>(2,0),0,-t.at<double>(0,0),
        -t.at<double>(1,0),t.at<double>(0,0),0);
    
    cout << "t^R = " << endl << t_x * R << endl;


}