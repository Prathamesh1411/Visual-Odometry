#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
    double ar = 1.0, br = 2.0, cr = 1.0;            // ground-truth values
    double ae = 2.0, be = -1.0, ce = 5.0;           // initial estimation
    int N = 100;                                    // num of data points
    double w_sigma = 1.0;                           // sigma of the noise
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                    // Random number generator

    vector<double> x_data, y_data;                  // the data
    for (int i = 0; i < N; i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // start Gauss-Newton iterations
    int iterations = 100;
    double cost = 0, lastCost = 0;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    for(iter = 0; iter < iterations; iter++){
        Matrix3d H = Matrix3d::Zero();            // Hessian = J^T W^{-1} J in Gauss-Newton
    }

}