#include <iostream>

struct Frame
{
    public:
        EIGEN_MAKE_ALIGNED_OPEERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

       
        unsigned long_id_ = 0;               // id of this frame
        unsigned long keyframe_id_ = 0;      // id of key frame
        bool is_keyframe_ = false;          
        double time_stamp_;
        SE3 pose_;
        std::mutex pose_mutex_;             // Pose data locker
        cv::Mat left_img_,right_img_;

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // correspoding features in right image, set to nullptr if no corresponding
    
    public:
        Frame() {}

        Fram(long id,double time_stamp,const SE3 &pose,const Mat &left,const Mat &right);

        // set and get pose,thread safe
        SE3 Pose()
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_
        }

        void SetPose(const SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;            
        }

        void SetKeyFrame();

        static std::shared_ptr<Frame> CreateFrame();

};
