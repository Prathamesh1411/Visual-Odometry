#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include <cmath>
#include <iostream>
#include <include/library_include.hpp>
#include <include/types_def.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace vslam
{

class VslamVisual
{
public:
    sensor_msgs::PointCloud2 feature_map_;
    ros::Publisher feature_map_publisher_;
    ros::Publisher fixed_pose_pub_;
    ros::Publisher pose_array_pub_;


public:
    VslamVisual() = default;
    VslamVisual(ros::NodeHandle &nh)
    {
        feature_map_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("vslam/feature_map", 1);
        fixed_pose_pub_ = nh.advertise<visualization_msgs::Marker>("fixed_pose", 100);
        pose_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("keyframes", 100);
    }

    int points_to_feature_map(const std::vector<cv::Point3f> &point_3d);
    int publish_feature_map(const std::vector<cv::Point3f> &point_3d);
    int publish_transform(const SE3 &T_c_w);
    void publish_fixed_pose(const Frame &frame);
    visualization_msgs::Marker create_pose_marker(const Frame &frame);
};
}

#endif /* visualization_hpp */
