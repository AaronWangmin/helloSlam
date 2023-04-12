
#include <iostream>

#include <eigen3/Eigen/Core>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// computer J_R^{-1}
Matrix6d JRInv(Sophus::SE3d e)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = Sophus::SO3d::hat(e.so3().log()); 
    J.block(0, 3, 3, 3) = Sophus::SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3::hat(e.so3().log());
    J = J * 0.5 + Matrix6d::Identity();
    return J;
}

// Lie algebra Vertex
typedef Eigen::Matrix<double, 6, 1> Vector6d;
class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override
    {
        double data[7];
        for (int i = 0; i < 7; i++)
        {
            is >> data[i];
        }
        setEstimate(SE3(
            Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
            Eigen::Vector3d(data[0], data[1], data[2])));
    }

    virtual bool write(ostream &os) const override
    {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translaion().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }

    virtual void setToOriginImpl()
    {
        _estimate = Sophus::SE3();
    }

    // left multiply update
    virtual void oplusImpl(const double* update)
    {
        Sophus::SE3d up(
            Sophus::SO3d(update[3],upate[4],update[5]),
            Eigen::Vector3d(update[0],update[1],update[2])
        );
        _estimate = up * _estimate;
    }
};

// edge: 2  lie algebra vertexs
class EdegeSE3Algebra : public g2o::BaseBinaryEdge<6, SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool read(istream& is)
    {
        double data[7];
        for(int i = 0; i < 7; i++)
            is >> data[i];
        Eigen::Quaternion q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(
            Sophus::SE3(q, Eigen::Vector3d(data[0], data[1], data[2]));            
        );
        for (int i = 0; i < information().rows() && is.good(); i++)
            for(int j = i; j < information().cols() && is.good(); j++)
            {
                is >> information()(i,j);
                if(i != j)
                    information()(j,i) = information()(i,j);
            }
        return true;        
    }

    bool write(ostream& os) const
    {
        VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
        VertexSE3LieAlgebra& v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        Sophus::SE3d m = _measurement;
        Eigen::Quaternion q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";
        // information matrix
        for(int i = 0; i < information().rows(); i++)
            for(int j = i; j < information().cols(); j++)
                os << information()(i,j) << " ";
        os << std::endl;
        return true;
    }
    
    // compute error 
    virtual void computeError()
    {
        Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
        Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // compute Joccobi
    virtual void linearizeOplus()
    {
        Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
        Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
        Matrix6d J = JRInv(Sophus::SE3d::exp(_error));
        _jacobianOplusXi = - J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }

};
