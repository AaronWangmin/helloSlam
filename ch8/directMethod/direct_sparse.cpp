
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
// #include <opencv2/ccalib.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d/features2d

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.hpp>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/block_solver.h>

struct Measurement
{
    Measurement(Eigen::Vector3d p, float g)
        : pos_world(p),
          grayscale(g) 
    {}

    Eigen::Vector3d pos_world;
    float grayscale;
};

inline Eigen::Vector3d project2Dto3D(
    int x, int y, int d,
    float fx, float fy, float cx, float cy, float scale)
{
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

inline Eigen::Vector2d project3Dto2D(
    float x, float y, float z,
    float fx, float fy, float cx, float cy)
{
    float u = fx * x / z + cx;
    float v = fy * y / z + cy;
    return Eigen::Vector2d(u, v);
}

bool poseEstimationDirector(
    const std::vector<Measurement>& measurements,
    cv::Mat* gray,
    Eigen::Matrix3f& intrinsics,
    Eigen::Isometry3d& Tcw);



class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge<1,double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3d point,
                        float fx, float fy, float cx, float cy,
                        cv::Mat* img)
        : x_world_(point),fx_(fx), fy_(fy), cx_(cx), cy_(cy),image_(img)
    {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_);
        float x = x_local[0] * fx_ / x_local[2] + cx_;
        float y = x_local[1] * fy_ / x_local[2] + cy_;

        if (x - 4 < 0 || x + 4 > image_->cols || y - 4 < 0 || y + 4 > image_->rows)
        {
            _error(0,0) = 0.0;
            this->setLevel(1);
        }
        else
        {
            _error(0,0) = this->getPixelValue(x,y) - _measurement;
        }        
    }

    virtual void linearizeOplus()
    {
        if (this->level() == 1)
        {
            this->_jacobianOplusXi = Eigen::Matrix<double,1,6>::Zero();
            return;
        }
        g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*>(this->_vertices[0]);
        Eigen::Vector3d xyz_trans = vtx->estimate().map(this->x_world_);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        float u = x * fx_ * invz + cx_;
        float v = y * fy_ * invz + cy_;

        Eigen::Matrix<double,2,6> jacobian_uv_ksia;

        jacobian_uv_ksia(0,0) = - x * y * invz_2 * fx_;
        jacobian_uv_ksia(0,1) = (1 + (x * x * invz_2)) * fx_;
        jacobian_uv_ksia(0,2) = - y * invz * fx_;
        jacobian_uv_ksia(0,3) = invz * fx_;
        jacobian_uv_ksia(0,4) = 0;
        jacobian_uv_ksia(0,5) = - x * invz_2 * fx_;

        jacobian_uv_ksia(1,0) = - (1 + y * y * invz_2) * fy_;
        jacobian_uv_ksia(1,1) = x * y * invz_2 * fy_;
        jacobian_uv_ksia(1,2) = x * invz * fy_;
        jacobian_uv_ksia(1,3) = 0;
        jacobian_uv_ksia(1,4) = invz * fy_;
        jacobian_uv_ksia(1,5) = - y * invz_2 * fy_;

        Eigen::Matrix<double,1,2> jacobain_pixel_uv;
        
        jacobain_pixel_uv(0,0) = (this->getPixelValue(u+1,v) - this->getPixelValue(u-1,v)) / 2;
        jacobain_pixel_uv(0,1) = (this->getPixelValue(u,v+1) - this->getPixelValue(u,v-1)) / 2;
        
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

protected:
    inline float getPixelValue(float x,float y)
    {
        uchar* data = &image_->data[int(y) * image_->step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[image_->step] +
            xx * yy * data[image_->step+1]
        );
    }

public:
    Eigen::Vector3d x_world_;
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0;
    cv::Mat* image_ = nullptr;

};

bool poseEstimationDirect(    
    const std::vector<Measurement>& measurements,
    cv::Mat* gray,
    Eigen::Matrix3f& K,
    Eigen::Isometry3d& Tcw)
{
    // -- first: create BlockSolve
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> BlockSolver_6_1;

    // -- second: create linearSolver
    std::unique_ptr<BlockSolver_6_1::LinearSolverType> linearSolver;          
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<BlockSolver_6_1::PoseMatrixType>>();

    // -- third: create solver
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolver_6_1>(std::move(linearSolver)));

    // -- 4th: create optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // -- 5th: add vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(),Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    // -- 6th: add edge
    int id = 1;
    for (Measurement m : measurements)
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
            m.pos_world,
            K(0,0), K(1,1), K(0,2), K(1,2), gray);
        edge->setVertex(0,pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setId(id++);
        optimizer.addEdge(edge);
    }
    std::cout << "edges in graph: " << optimizer.edges().size() << std::endl;

    // 7th: optimize
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();  
    
}