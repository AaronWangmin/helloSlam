class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long,MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long,Frame::Ptr> KeyframesType;
    
    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);

    LandmarksType GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        return landmarks_;
    }

    KeyframeTypes GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        return keyframes_;
    }

    LandmarksTypes GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        return active_landmars_;
    }

    KeyframeType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    void ClearMap();

private:
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframeType keyframes_;
    KeyframeType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    //setting
    int num_active_keyframes_ = 7;
};


