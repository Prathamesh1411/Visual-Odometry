#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_

#include <cmath>
#include <iostream>
#include <include/library_include.hpp>
#include <include/types_def.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <include/visualization.hpp>
#include <include/optimization.hpp>
#include <include/map.hpp>

namespace vslam
{

enum TrackState
{
    Init,
    Track,
    Lost
};

class VO
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame frame_last_;
    Frame frame_current_;
    Map &my_map_;

    std::string dataset_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_crosscheck_;

    int num_inliers_ = 0;

    SE3 T_c_l_ = SE3(); // T_current(camera)_last(camera)
    SE3 T_c_w_ = SE3(); // T_current(camera)_world

    int seq_ = 1; //sequence number

    //visualization module
    VslamVisual my_visual_;

    TrackState state_ = Init; // current tracking state
    int num_lost_ = 0;        // number of continuous lost frames

    int curr_keyframe_id_ = 0;
    int curr_landmark_id_ = 0;

    bool if_rviz_;

public:
    VO(ros::NodeHandle &nh, Map &map);
    VO(std::string dataset, ros::NodeHandle &nh, Map &map);
    int read_img(int id, cv::Mat &left_img, cv::Mat &right_img);
    int disparity_map(const Frame &frame, cv::Mat &disparity);
    bool initialization();
    bool tracking(bool &if_insert_keyframe);
    int feature_detection(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    int feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &feature_matches);
    std::vector<bool> set_ref_3d_position(std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, Frame &frame);
    void motion_estimation(Frame &frame);
    bool check_motion_estimation();
    void move_frame();
    void write_pose(const Frame &frame);
    void rviz_visualize();
    void adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints, const int num);
    bool pipeline(bool &if_insert_keyframe);
    bool insert_key_frame(bool check, std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
};


} // namespace vslam

#endif /* visual_odometry_hpp */