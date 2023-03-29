#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;    
    enum VOState
    {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState state_;     // current VO Status
    Map::Ptr map_;      // map with all frame and map points

    Frame::Ptr ref_;    // reference key-frame
    Frame::Ptr curr_;   // current frame

    cv::Ptr<cv::ORB> orb_;  //orb detector and computer
    std::vector<cv::KeyPoint> keypoints_curr_;  // keypoints in current frame
    cv::Mat descriptors_curr_;                  // descriptor in current frame

    cv::FlannBasedMatcher matcher_flann_;
    std::vector<MapPoint::Ptr> match_3dpts_;
    std::vector<int> match_2dkp_index_;

    Sophus::SE3d T_c_w_estimated_;   // the estimated pose of current frame
    int num_inliers_;       // number of inlier features in icp
    int num_lost_;          // number of lost times

    // parameters
    int num_of_features_;
    double scale_factor_;
    int level_pyramid_;
    float match_ratio_;
    int max_num_lost_;
    int min_inliers_;
    double key_frame_min_rot_;   // minimal rotation of two key-frame
    double key_frame_min_trans_; // minimal translation of two key-frame
    double map_point_erase_ratio_;

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame);

protected:  
    // inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void optimizeMap();

    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose();
    bool checkKeyFrame();

    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point);

};

}

#endif