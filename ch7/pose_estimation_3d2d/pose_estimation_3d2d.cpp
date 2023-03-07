
// #include "pose_estimation_3d2d.h"

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include "g2o/types/slam3d/vertex_pointxyz.h"


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


int main(int argc,char** argv)
{
    if(argc != 5)
    {
        cout << "usage: pose_estimation_3d2d img_1 img_2,depth_1 depth_2" << endl;
    }

    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);

    Mat d1 = imread(argv[2],CV_LOAD_IMAGE_UNCHANGED);
    Mat d2 = imread(argv[4],CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<Point3d> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m : matches)
    {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if(d == 0) // bad depth 
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    cout << "3d-2d pairs: " << pts_2d.size() << endl;

    Mat r,t;
    solvePnP(pts_3d,pts_2d,K,Mat(),r,t,false,SOLVEPNP_EPNP);
    Mat R;
    cv::Rodrigues(r,R);

    cout << "R= " << endl << R << endl;
    cout << "t= " << endl << t << endl;

    cout << "calling bundle adjustment" << endl;
    

}





void find_feature_matches(
    const Mat &img_1, 
    const Mat &img_2,
    vector<KeyPoint> &keypoints_1,
    vector<KeyPoint> &keypoints_2,
    vector<DMatch> &matches)
{
    // -- initial
    // vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptor_1,descriptor_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // -- First: detect Orented FAST
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    // -- Second: compute BRIEF descriptor by Oriented FAST
    descriptor->compute(img_1,keypoints_1,descriptor_1);
    descriptor->compute(img_2,keypoints_2,descriptor_2);

    // -- Third: match the BRIEF descriptor of two images, using Hamming distance
    // vector<DMatch> matches;
    matcher->match(descriptor_1,descriptor_2,matches);

    // -- 4th: choose matches
    double min_dist = 10000, max_dist = 0;

    // find the max/min distance
    for (int i = 0; i < descriptor_1.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    // only for funny
    min_dist = min_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance < m2.distance;})->distance;
    max_dist = max_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance < m2.distance;})->distance;

    printf("-- Max dist: %f \n",max_dist);
    printf("-- Min dist: %f \n",min_dist);   

    // the distance is more than 2*min_dist, or 30
    vector<DMatch> good_matches;
    for (int i = 0; i < descriptor_1.rows; i++)
    {
        if (matches[i].distance < max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }        
    } 
}

Point2d pixel2cam(
    const Point2d &p, 
    const Mat &K)
{
    return Point2d(
        (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
        (p.y - K.at<double>(1,2) / K.at<double>(1,1))
    );
}

void bundleAdjustment(
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat &K,
    Mat &R,
    Mat &t)
{
    // initial g2o
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);


    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block; // pose 6-dim, landmark 3-dim
    // typedef g2o::LinearSolverCSparse<Block::PoseMatrixType> LinearSolver;
    // auto solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<Block>(g2o::make_unique<LinearSolver>()));

    // Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    // Block *solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
    // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr));
    // unique_ptr<Block::LinearSolverType> linearSolver( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    // unique_ptr<Block> solver_ptr( new Block(move(linearSolver)));
    // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));
    

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;    
    R_mat << 
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
    pose->setId(0);
    pose->setFixed(true);
    pose->setEstimate(g2o::SE3Quat(
            R_mat,
            Eigen::Vector3d(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0))
    ));
    optimizer.addVertex(pose);
    // for (int i = 0; i < 2; i++)
    // {
    //     g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    //     pose->setId(i);
    //     if (i == 0)
    //     {
    //         pose->setFixed(true);
    //     }
    //     pose->setEstimate(g2o::SE3Quat());
    //     optimizer.addVertex(pose);
    // }

    int index = 1;
    for (const Point3f p : points_3d) // landmarks
    {        
        g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
        point->setId(index++);
        point->setMarginalized(true);
        point->setEstimate(Eigen::Vector3d(p.x,p.y,p.z));
        optimizer.addVertex(point);        
    }

    // parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0,0),Eigen::Vector2d(K.at<double>(0,2),K.at<double>(1,2)),0.0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges
    index = 1;
    for (const Point2f p : points_2d)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0,dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(index)));
        edge->setVertex(1,pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0,0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }  
    

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 -t1);
    cout << "optimization cost time: " << time_used.count() << " seconds. " << endl;

    cout << endl << "after optimization: " << endl;
    cout << "T= " << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
    
}


