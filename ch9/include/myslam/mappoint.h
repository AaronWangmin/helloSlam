#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
class Frame;

class MapPoint
{
public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    static unsigned long factory_id_;
    bool good_;
    Vector3d pos_;
    Vector3d norm_;     // Normal of viewing direction
    Mat descriptor_;    // Descriptor for matching

    list<Frame*> observed_frames_;  // key-frames that can observe this point

    int matched_times_;     // being an inliner in pose estimation
    int visible_times_;     // being visible in current frame

    MapPoint();

    MapPoint(
        unsigned long id,
        const Vector3d& position,
        const vector3d& norm,
        Frame* frame = nullptr,
        const Mat& descriptor = Mat());
    
    inline cv::Point3f getPositionCV() const
    {
        return cv::Point3f(pos_(0,0), pos(0,1), pos(0,2));
    }

    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(
        const Vector3d& pos_world,
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame )
};


}


#endif