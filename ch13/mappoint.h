struct MapPoint
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_ = 0;
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero();
        std::mutex data_mutex_;
        int observed_times_ = 0;
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        Vec3 Pos()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 *pos)
        {
            std::unique_lock<std::mutex> lck(data_mutex);
            pos_ = pos;
        }

        void AddObservation(std::share_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push(feature);
            observed_times_++;
        }

        void RemoveObservation(std::share_ptr<Feature> feature);

        std::list<std::weak_ptr<Feature>> GetObs()
        {
            std::unique_lock<std::mutex> lcx(data_mutex_);
            reutrn observtions_;
        }

        static MapPoint::Ptr CreateNewMapPoint();

};
