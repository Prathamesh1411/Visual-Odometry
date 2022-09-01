#include <cmath>
#include <iostream>
#include <include/library_include.hpp>
#include <include/types_def.hpp>
#include <include/map.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <include/visualization.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <include/optimization.hpp>

namespace vslam
{

void VertexPose::oplusImpl(const double *update)
{
    Eigen::Matrix<double, 6, 1> update_se3;
    update_se3 << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_se3)*_estimate; 
}

void VertexXYZ::oplusImpl(const double *update)
{
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
}

void EdgeProjection::computeError()
{
    const VertexPose* v0 = static_cast<VertexPose*> (_vertices[0]);
    const VertexXYZ* v1 = static_cast<VertexXYZ*> (_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pos_pixel = K_ * (T * v1->estimate());
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjection::linearizeOplus()
{
    const VertexPose *v0 = static_cast<VertexPose*>(_vertices[0]);
    const VertexXYZ* v1 = static_cast<VertexXYZ*> (_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pw = v1->estimate();
    Eigen::Vector3d pos_cam = T * pw;
    double fx = K_(0, 0);
    double fy = K_(1, 1);
    double cx = K_(0, 2);
    double cy = K_(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
        fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
        -fy * X * Zinv;
    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.rotationMatrix();
}

void PoseOnlyEdgeProjection::computeError()
{
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = K_ * (T * point_3d_);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
}

void PoseOnlyEdgeProjection::linearizeOplus()
{
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * point_3d_;
    double fx = K_(0, 0);
    double fy = K_(1, 1);
    double cx = K_(0, 2);
    double cy = K_(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;
    _jacobianOplusXi
        << -fx / Z,
        0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
}

void optimize_map(std::unordered_map<unsigned long, Frame> &keyframes, std::unordered_map<unsigned long, Landmark> &landmarks,
                const cv::Mat &K, bool if_update_map, bool if_update_landmark, int num_ite)
{
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
}
} //namespace vslam
