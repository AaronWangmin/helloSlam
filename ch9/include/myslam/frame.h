#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
// forward declare
class MapPoint;

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_;
    double time_stamp_;
    Sophus::SE3d T_c_w_;          
    Camera::Ptr camera_;
    Mat color_, depth_;

    bool is_key_frame_;

public:
    Frame();
    Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr = nullptr,Mat color = Mat(), Mat depth = Mat() );
    ~Frame();

    static Frame::Ptr createFrame();

    // find the depth in depth map
    double findDepth(const cv::KeyPoint& kp);

    // Get Camera Center
    Vector3d getCameraCenter() const;

    void setPose(const SE3& T_c_w);
    
    // check if a point is in this frame
    bool isInFrame(const Vector3d& pt_world);

};

}

#endif