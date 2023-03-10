#include "pose_estimation_3d3d.h"

int main(int argc,char** argv)
{
    const string img1 = "/home/ubuntu/slambook/ch7/pose_estimation_3d3d/1.png";
    const string img2 = "/home/ubuntu/slambook/ch7/pose_estimation_3d3d/2.png";
    const string depth1 = "/home/ubuntu/slambook/ch7/pose_estimation_3d3d/1_depth.png";
    const string depth2 = "/home/ubuntu/slambook/ch7/pose_estimation_3d3d/2_depth.png";
    
    PoseEstimation3D3D poseEstimation3D3D(img1,img2,depth1,depth2);
    
    poseEstimation3D3D.find_feature_matches();
    poseEstimation3D3D.build3DPoints();
    poseEstimation3D3D.pose_estimation_3d3d();
}   


PoseEstimation3D3D::PoseEstimation3D3D(
        const string &imgFileName_1,
        const string &imgFileName_2,
        const string &depthFileName_1,
        const string &depthFileName_2)
{
    this->img_1 = cv::imread(imgFileName_1,CV_LOAD_IMAGE_COLOR);
    this->img_2 = cv::imread(imgFileName_2,CV_LOAD_IMAGE_COLOR);

    this->depth_1 = cv::imread(depthFileName_1,CV_LOAD_IMAGE_UNCHANGED);
    this->depth_2 = cv::imread(depthFileName_2,CV_LOAD_IMAGE_UNCHANGED);
}

void PoseEstimation3D3D::find_feature_matches()
{
    // -- initial
    cv::Mat descriptors_1,descriptors_2;    
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");                                                                            

    detector->detect(this->img_1,this->keypoints_1);
    detector->detect(this->img_2,this->keypoints_2);

    descriptor->compute(this->img_1,this->keypoints_1,descriptors_1);
    descriptor->compute(this->img_2,this->keypoints_2,descriptors_2);

    std::vector<cv::DMatch> match;
    matcher->match(descriptors_1,descriptors_2,match);

    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    printf("--Max dist: %f \n",max_dist);
    printf("--Min dist: %f \n",min_dist);

    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            this->matches.push_back(match[i]);
        }        
    }

    std::cout << "total matches points: " << this->matches.size() << std::endl;
}

void PoseEstimation3D3D::pose_estimation_3d3d(
    // const cv::Mat &img_1,
    // const cv::Mat &img_2,
    // std::vector<cv::KeyPoint> &keypoints_1,
    // std::vector<cv::KeyPoint> &keypoints_2,
    // std::vector<cv::DMatch> &matches
)
{
    // -- 1th: munus center of mass
    cv::Point3f p1,p2;
    int N = this->pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += this->pts1[i];
        p2 += this->pts2[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);
    std::vector<cv::Point3f> q1(N),q2(N);
    for (int i = 0; i < N; i++)
    {
        q1[i] = this->pts1[i] - p1;
        q2[i] = this->pts2[i] -p2;
    }

    // -- 2th: compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * 
            Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    std::cout << "W= " << W << endl; 

    // --3th: SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x,2) *= -1;
        }        
    }

    std::cout << "U= " << U << endl;
    std::cout << "V= " << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * 
        Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (cv::Mat_<double>(3,3) << 
        R_(0,0),R_(0,1),R_(0,2),
        R_(1,0),R_(1,1),R_(1,2),
        R_(2,0),R_(2,1),R_(2,2)
        );
    t = (cv::Mat_<double>(3,1) << t_(0,0), t_(1,0), t_(2,0));

    std::cout << "ICP via SVD result: " << std::endl;
    std::cout << "R = " << R << std::endl;
    std::cout << "t = " << t << std::endl;
    std::cout << "R_inv = " << R.t() << std::endl;
    std::cout << "t_inv = " << -R.t() * t << std::endl;

}

void PoseEstimation3D3D::build3DPoints()
{
    for (cv::DMatch m : this->matches)
    {
        std::cout << "build 3d-points test.............." << this->matches.size() << std::endl
            << this->keypoints_1.size() << std::endl;
        std::cout << this->depth_1.size() << std::endl << "depth_2 " << this->depth_2.size() << std::endl;
        
        std::cout << this->img_1.size() << std::endl << "img_2 " << this->img_2.size() << std::endl;

        ushort d1 = this->depth_1.ptr<unsigned short>(int(this->keypoints_1[m.queryIdx].pt.y))
            [int(this->keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = this->depth_2.ptr<unsigned short>(int(this->keypoints_2[m.trainIdx].pt.y))
            [int(this->keypoints_2[m.trainIdx].pt.x)];
        if (d1 == 0 || d2 == 0) continue; // bad depth

        cv::Point2d p1 = this->pixel2cam(this->keypoints_1[m.queryIdx].pt,this->K);
        cv::Point2d p2 = this->pixel2cam(this->keypoints_2[m.trainIdx].pt,this->K);
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;
        
        this->pts1.push_back(cv::Point3f(p1.x * dd1, p1.y * dd1, dd1));
        this->pts2.push_back(cv::Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    std::cout << "total 3d-3d pairs: " << this->pts1.size() << std::endl;
    
}

void PoseEstimation3D3D::bundleAdjunstment()
{
    // -- initial g2o
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    g2o::OptimizationAlgorithmLevenberg* solver = 
        new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<
                g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));
    
    optimizer.setAlgorithm(solver);

    // vertex    
    g2o::VertexSE3* pose = new g2o::VertexSE3();
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
             R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
             R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
    pose->setEstimate(g2o::SE3Quat(
            R_mat,
            Eigen::Vector3d(t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2))));
    optimizer.addVertex(pose);

    // edges
    int index = 1;
    std::vector<g2o::Edge_V_V_GICP*> edges;
    for (size_t i = 0; i < this->pts1.size(); i++)
    {
        g2o::Edge_V_V_GICP* edge = new g2o::Edge_V_V_GICP();
        edge->setVertex(0,this->pts1[i]);
        edge->setVertex(1,this->pts2[i]);

    }
    









}

cv::Point2d PoseEstimation3D3D::pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
    return cv::Point2d(
        (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
        (p.y - K.at<double>(1,2)) / K.at<double>(1,1));
}





