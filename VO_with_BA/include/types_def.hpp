#ifndef TYPES_DEF_H_
#define TYPES_DEF_H_

#include <cmath>
#include <iostream>
#include <include/library_include.hpp>

namespace vslam
{

struct Frame;
struct Landmark;
struct Feature;

struct Feature
{
public:
    int feature_id_;
    int frame_id_;
    int landmark_id_ = -1;
    cv::KeyPoint keypoint_; // 2d position in pixel frame
    cv::Mat descriptor_; // feature descriptor of this landmark
    bool is_inlier = false;

    Feature(){}
    Feature(int feature_id, int frame_id, cv::KeyPoint keypoint, cv::Mat descriptor)
        : feature_id_(feature_id), frame_id_(frame_id), keypoint_(keypoint), descriptor_(descriptor) {}
};

struct Frame
{
public:
    int frame_id_;
    cv::Mat left_img_, right_img_;
    cv::Mat disparity_;

    SE3 T_c_w_ = SE3(); //

    bool is_keyframe_;
    int keyframe_id_;
    std::vector<Feature> features_;

    //camera intrinsics
    double fx_ = 718.856, fy_ = 718.856, cx_ = 607.1928, cy_ = 185.2157;
    double b_ = 0.573;

    Frame() = default;
    Frame(int frame_id, double timestamp, const cv::Mat &left, const cv::Mat &right)
        : frame_id_(frame_id), left_img_(left), right_img_(right){}

    Eigen::Vector3d find_3d(const cv::KeyPoint &kp, Eigen::Vector3d &relative_pt3d);

    void fill_frame(SE3 T_c_w, bool is_keyframe, int keyframe_id);
};

struct Observation
{
    int keyframe_id_;
    int feature_id_;
    bool to_delete = false;

    Observation(int keyframe_id, int feature_id)
        : keyframe_id_(keyframe_id), feature_id_(feature_id) {}
};

struct Landmark
{

public:
    int landmark_id_;
    cv::Point3f pt_3d_;      // 3d position in the world frame;
    cv::Mat descriptor_;     // feature descriptor of this landmark
    int observed_times_ = 1; // number of times being observed
    std::vector<Observation> observations_;
    bool is_inlier = true;
    bool reliable_depth_ = false;

public:
    Landmark() {}

    Landmark(int landmark_id, cv::Point3f pt_3d, cv::Mat descriptor, bool reliable_depth, Observation observation)
        : landmark_id_(landmark_id), pt_3d_(pt_3d), descriptor_(descriptor), reliable_depth_(reliable_depth)
    {
        observations_.push_back(observation);
    }

    Eigen::Vector3d to_vector_3d()
    {
        Eigen::Vector3d pos_vec;
        pos_vec << pt_3d_.x, pt_3d_.y, pt_3d_.z;
        return pos_vec;
    }
};

} // namespace vslam



#endif /* types_def_hpp */