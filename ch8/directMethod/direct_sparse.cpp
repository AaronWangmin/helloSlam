
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/ccalib.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d/features2d.hpp>


#include <g2o/core/base_vertex.hpp>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

struct Measurement
{
    Measurement(Eigen::Vector3d p, float g)
        : pose_world(p),
          grayscale(g) 
    {}

    Eigen::vector3d pos_world;
    float grayscale;
};

inline Eigen::Vector3d project2Dto3D(
    int x, y, d,
    float fx, fy, cx, cy, scale)
{
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

inline Eigen::Vector2d project3Dto2D(
    float x, y, z,
    float fx, fy, cx, cy
)
{
    float u = fx * x / z + cx;
    float v = fy * y / z + cy;
    return Eigen::Vector2d(u, v);
}

bool poseEstimationDirector(
    const vector<Measurement>& measurements,
    cv::Mat* gray,
    Eigen::Matrix3f& intrinsics,
    Eigen::Isometry3d& Tcw
);

class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge<1,double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3d point,
                        float fx, fy, cx, cy,
                        cv::Mat* img)
        : x_word_(point),fx_(fx), fy_(fy), cx_(cx), cy_(cy),image_(image)
    {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_);
        float x = x_local[0] * fx_ / x_local[2] + cx_;
        float y = x_local[1] * fy_ / x_local[2] + cy_;

        if (x - 4 < 0 || x + 4 > image_->cols || y - 4 < 0 || y + 4 ? image_->rows)
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
    virtual boll write(std::ostream& out) {}

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