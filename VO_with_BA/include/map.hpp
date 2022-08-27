#ifndef MAP_H_
#define MAP_H_

#include <cmath>
#include <iostream>
#include <include/library_include.hpp>
#include <include/types_def.hpp>
#include <include/visualization.hpp>

namespace vslam
{
    
struct Map
{
public:
    std::unordered_map<unsigned long, Frame> keyframes_;
    std::unordered_map<unsigned long, Landmark> landmarks_;
    const int num_keyframes_ = 10;
    int current_keyframe_id_ = 0;
    VslamVisual my_visual_;
    bool if_write_pose_;
    bool if_rviz_;

public:
    Map(ros::NodeHandle &nh) : my_visual_(nh)
    {
        nh.getParam("/if_write_pose", if_write_pose_);
        nh.getParam("/if_rviz", if_rviz_);
    }
    int insert_keyframe(Frame frame_to_add);
    int insert_landmark(Landmark landmark_to_add);
    int remove_keyframe();
    int clean_map();
    void publish_keyframes();
    void write_pose(const Frame &frame);
    void write_remaining_pose();
};

} // namespace vslam

#endif /* map_hpp */