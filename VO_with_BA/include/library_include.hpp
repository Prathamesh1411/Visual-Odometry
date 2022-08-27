#ifndef LIBRARY_INCLUDE_H_
#define LIBRARY_INCLUDE_H_

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#endif /*LIBRARY_INCLUDE_H_*/