#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc,char **argv)
{
    double ar = 1.0,br = 2.0, cr = 1.0;
    double ae = 2.0,be = -1.0,ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;

    vector<double> x_data,y_data;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
        
    }

    int iterations = 100;
    double cost = 0,lastCost = 0;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)
    {
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        cost = 0;

        for (int i = 0; i < N; i++)
        {
            double xi = x_data[i],yi = y_data[i];
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Vector3d J;
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = -exp(ae * xi * xi + be * xi + ce);

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }

        Vector3d dx = H .ldlt().solve(b);
        if(isnan)

        
    }
    

    
}