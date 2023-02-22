#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc,char **argv)
{
    Matrix3d R = AngleAxisd(M_PI / 2,Vector3d(0,0,1)).toRotationMatrix();    
    Quaterniond q(R);

    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    cout << "SO3 from matrix: \n " << SO3_R.matrix() << endl;
    cout << "SO3 from quatenion: \n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() <<endl;

    Vector3d update_so3(1e-4,0,0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

    cout << "*********************************************" << endl;
    


    return 0;


}







