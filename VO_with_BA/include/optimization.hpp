#ifndef OPTIMIZATION_H_
#define OPTIMIZATION_H_

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
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_binary_edge.h>

namespace vslam
{

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> G2OVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> G2OVector3d;

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override {_estimate = Sophus::SE3d();}
    virtual void oplusImpl(const double *update) override;
    virtual bool read(std::istream &in) override {}
    virtual bool write(std::ostream &out) const override {}
};

//define the landmark
class VertexXYZ : public g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = Eigen::Matrix<double, 3, 1>::Zero(); }
    virtual void oplusImpl(const double *update) override;
    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }
};

// define the edge
// parameter: [dimension of measurement, data type, vertex type]
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjection(const Eigen::Matrix3d &K) : K_(K) {}

    /*! \brief compute the error
    *
    * 1. get the current optimized pose in this iteration
    *       it is the _estimate in vertice class
    *       it is already in SE3 because exp has been done in update function
    * 2. calculate error by e = u - (1/s)KTP
    */
    virtual void computeError() override;

    /*! \brief give the analytic solution of jacobian
    */
    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}

public:
    Eigen::Matrix3d K_;
};

// pose only projection
class PoseOnlyEdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PoseOnlyEdgeProjection(const Eigen::Vector3d &point_3d, const Eigen::Matrix3d &K) : point_3d_(point_3d), K_(K) {}

    /*! \brief compute the error
    */
    virtual void computeError() override;

    /*! \brief give the analytic solution of jacobian
    */
    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}

public:
    Eigen::Vector3d point_3d_;
    Eigen::Matrix3d K_;
};

/*! \brief optimize the keyframes and landmarks in the map
 * 
 *  \param keyframes - a reference of keyframes in the map
 *  \param landmarks - a reference of landmarks in the map
 *  \param K - Camera intrinsic parameters
 *  \param if_update_map - whether modify the poses and landmarks according to optimization results
 *                          if set to true, the map will be updated
 *  \param if_update_landmark - whether modify the landmarks in the map
 *                          if set to false, only poses will be updated
 *  \param num_ite - number of iterations
*/
void optimize_map(std::unordered_map<unsigned long, Frame> &keyframes,
                  std::unordered_map<unsigned long, Landmark> &landmarks,
                  const cv::Mat &K, bool if_update_map, bool if_update_landmark, int num_ite);

/*! \brief optimize only the poses of the keyframes in the map
 * 
 *  \param keyframes - a reference of keyframes in the map
 *  \param landmarks - a reference of landmarks in the map
 *  \param K - Camera intrinsic parameters
 *  \param if_update_map - whether modify the poses according to optimization results
 *                          if set to true, the map will be updated
 *  \param num_ite - number of iterations
*/
void optimize_pose_only(std::unordered_map<unsigned long, Frame> &keyframes,
                        std::unordered_map<unsigned long, Landmark> &landmarks,
                        const cv::Mat &K, bool if_update_map, int num_ite);

} // namespace vslam


#endif /* optimization_hpp */