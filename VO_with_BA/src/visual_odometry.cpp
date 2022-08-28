#include <include/visual_odometry.hpp>
#include <include/library_include.hpp>
#include <include/types_def.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <include/visualization.hpp>
#include <include/optimization.hpp>
#include <include/map.hpp>

namespace vslam
{

VO::VO(ros::NodeHandle &nh, Map &map) : my_visual_(nh), my_map_(map)
{
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    nh.getParam("/if_rviz", if_rviz_);
}

VO::VO(std::string dataset, ros::NodeHandle &nh, Map &map) : my_visual_(nh), my_map_(map)
{
    dataset_ = dataset;
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    nh.getParam("/if_rviz", if_rviz_);
}

int VO::read_img(int id, cv::Mat &left_img, cv::Mat &right_img)
{
    std::string left_address, right_address, image_name;
    image_name = std::to_string(id);
    image_name = std::string(6 - image_name.length(), '0') + image_name;
    left_address = this->dataset_ + "image_0/" + image_name + ".png";
    right_address = this->dataset_ + "image_1/" + image_name + ".png";

    left_img = cv::imread(left_address, cv::IMREAD_GRAYSCALE);
    right_img = cv::imread(right_address, cv::IMREAD_GRAYSCALE);

    if (!left_img.data)
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    return 0;
}

int VO::feature_detection(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    if (!img.data)
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    detector_->detect(img, keypoints);
    adaptive_non_maximal_suppresion(keypoints, 500);

    descriptor_->compute(img, keypoints, descriptors);

    cv::Mat outimg1;
    cv::drawKeypoints(img, keypoints, outimg1);
    cv::imshow("ORB features", outimg1);
    cv::waitKey(1);

    return 0;
}

void VO::adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints, const int num)
{
    // if number of keypoints is already lower than the threshold, return
    if (keypoints.size() < num)
    {
        return;
    }

    std::sort(keypoints.begin(), keypoints.end(), [&](const cv::KeyPoint &lhs, const cv::KeyPoint &rhs){
        return lhs.response > rhs.response;
    });

    // vector for store ANMS points
    std::vector<cv::KeyPoint> ANMSpt;

    std::vector<double> rad_i;
    rad_i.resize(keypoints.size());

    std::vector<double> rad_i_sorted;
    rad_i_sorted.resize(keypoints.size());

    // robust coefficient: 1/0.9 = 1.1
    const float c_robust = 1.11;

    for(int i = 0; i < keypoints.size(); ++i)
    {
        const float response = keypoints.at(i).response * c_robust;

        // maximum finit number of double
        double radius = std::numeric_limits<double>::max();

        for(int j = 0; j < i && keypoints.at(j).response > response; ++j)
        {
            radius = std::min(radius, cv::norm(keypoints.at(i).pt - keypoints.at(j).pt));
        }

        rad_i.at(i) = radius;
        rad_i_sorted.at(i) = radius;
    }

    std::sort(rad_i_sorted.begin(), rad_i_sorted.end(), [&](const double &lhs, const double &rhs) {
        return lhs > rhs;
    });

    // find the final radius
    const double final_radius = rad_i_sorted.at(num - 1);
    for (int i = 0; i < rad_i.size(); ++i)
    {
        if (rad_i.at(i) >= final_radius)
        {
            ANMSpt.push_back(keypoints.at(i));
        }
    }

    // swap address to keypoints
    keypoints.swap(ANMSpt);
}

int VO::disparity_map(const Frame &frame, cv::Mat &disparity)
{
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

    cv::Mat disparity_sgbm;
    sgbm->compute(frame.left_img_, frame.right_img_, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    return 0;
}

std::vector<bool> VO::set_ref_3d_position(std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints,
                                        cv::Mat &descriptors, Frame &frame)
{
    pts_3d.clear();
    cv::Mat descriptors_last_filtered;
    std::vector<cv::KeyPoint> keypoints_last_filtered;
    std::vector<bool> reliable_depth;

    for(size_t i = 0; i < keypoints.size(); i++)
    {
        Eigen::Vector3d relative_pos_3d;
        Eigen::Vector3d pos_3d = frame.find_3d(keypoints.at(i), relative_pos_3d);
        if (relative_pos_3d(2) > 10 && relative_pos_3d(2) < 400)
        {
            pts_3d.push_back(cv::Point3f(pos_3d(0), pos_3d(1), pos_3d(2)));
            descriptors_last_filtered.push_back(descriptors.row(i));
            keypoints_last_filtered.push_back(keypoints.at(i));

            if (relative_pos_3d(2) < 40)
            {
                reliable_depth.push_back(true);
            }
            else
            {
                reliable_depth.push_back(false);
            }
        }
    }

    descriptors = descriptors_last_filtered;
    keypoints = keypoints_last_filtered;

    return reliable_depth;
}

int VO::feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &feature_matches)
{
    feature_matches.clear();
    std::vector<cv::DMatch> matches_crosscheck;
    matcher_crosscheck_->match(descriptors_1, descriptors_2, matches_crosscheck);

    auto min_max = minmax_element(matches_crosscheck.begin(), matches_crosscheck.end(), [](const auto &lhs, const auto &rhs){
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;

    double frame_gap = frame_current_.frame_id_ - frame_last_.frame_id_;
    for (int i = 0; i < matches_crosscheck.size(); i++)
    {
        if (matches_crosscheck.at(i).distance <= std::max(2.0 * min_element->distance, 30.0 * frame_gap))
        {
            feature_matches.push_back(matches_crosscheck.at(i));
        }
    }

    return 0;
}

void VO::motion_estimation(Frame &frame)
{
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for (int i = 0; i < frame.features_.size(); i++)
    {
        int landmark_id = frame.features_.at(i).landmark_id_;
        if (landmark_id == -1)
        {
            std::cout << "No landmark associated!" << std::endl;
        }

        pts3d.push_back(my_map_.landmarks_.at(landmark_id).pt_3d_);
        pts2d.push_back(frame.features_.at(i).keypoint_.pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << frame.fx_, 0, frame.cx_,
                 0, frame.fy_, frame.cy_,
                 0, 0, 1);

    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

    num_inliers_ = inliers.rows;

    cv::Mat SO3_R_cv;
    cv::Rodrigues(rvec, SO3_R_cv);
    Eigen::Matrix3d SO3_R;
    SO3_R << SO3_R_cv.at<double>(0, 0), SO3_R_cv.at<double>(0, 1), SO3_R_cv.at<double>(0, 2),
        SO3_R_cv.at<double>(1, 0), SO3_R_cv.at<double>(1, 1), SO3_R_cv.at<double>(1, 2),
        SO3_R_cv.at<double>(2, 0), SO3_R_cv.at<double>(2, 1), SO3_R_cv.at<double>(2, 2);

    T_c_w_ = SE3(
        SO3_R,
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx, 0);
        frame.features_.at(index).is_inlier = true;
    }

    // delete outliers
    frame.features_.erase(std::remove_if(
                              frame.features_.begin(), frame.features_.end(),
                              [](const Feature &x) {
                                  return x.is_inlier == false;
                              }),
                          frame.features_.end());
}


bool VO::check_motion_estimation()
{
    if(num_inliers_ < 10)
    {
        std::cout << "Frame id: " << frame_last_.frame_id_ << " and " << frame_current_.frame_id_ << std::endl;
        std::cout << "Rejected - inliers not enough: " << num_inliers_ << std::endl;
        return false;
    }

    Sophus::Vector6d displacement = T_c_l_.log();
    double frame_gap = frame_current_.frame_id_ - frame_last_.frame_id_;
    if (displacement.norm() > (5.0 * frame_gap))
    {
        std::cout << "Frame id: " << frame_last_.frame_id_ << " and " << frame_current_.frame_id_ << std::endl;
        std::cout << "Rejected - motion is too large: " << displacement.norm() << std::endl;
        return false;
    }

    return true;
}

bool VO::insert_key_frame(bool check, std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    if ((num_inliers_ >= 80 && T_c_l_.angleY() < 0.03) || check == false){ return false; }
    frame_current_ .is_keyframe_ = true;
    frame_current_.keyframe_id_ =  curr_keyframe_id_;

    for(int i = 0; i < frame_current_.features_.size(); i++)
    {
        int landmark_id = frame_current_.features_.at(i).landmark_id_;
        my_map_.landmarks_.at(landmark_id).observed_times_++;
        Observation observation(frame_current_.keyframe_id_, frame_current_.features_.at(i).feature_id_);
        my_map_.landmarks_.at(landmark_id).observations_.push_back(observation);
    }

    disparity_map(frame_current_, frame_current_.disparity_);
    std::vector<bool> reliable_depth = set_ref_3d_position(pts_3d, keypoints, descriptors, frame_current_);

    // calculate the world coordinate
    // no relative motion any more

    int feature_id = frame_current_.features_.size();
    for(int i = 0; i < keypoints.size(); i++)
    {
        bool exist = false;
        for(auto &feat : frame_current_.features_)
        {
            if(feat.keypoint_.pt.x == keypoints.at(i).pt.x && feat.keypoint_.pt.y == keypoints.at(i).pt.y)
            {
                exist = true;
                if((my_map_.landmarks_.at(feat.landmark_id_).reliable_depth_ = false) && (reliable_depth.at(i) == true))
                {
                    my_map_.landmarks_.at(feat.landmark_id_).pt_3d_ = pts_3d.at(i);
                    my_map_.landmarks_.at(feat.landmark_id_).reliable_depth_ = true;
                }
            }
        }

        if(exist == false)
        {
            // add this feature
            // put the features into the frame with feature_id, frame_id, keypoint, descriptor
            // build the connection from feature to frame
            Feature feature_to_add(feature_id, frame_current_.frame_id_, keypoints.at(i), descriptors.row(i));
            feature_to_add.landmark_id_ = curr_landmark_id_;
            frame_current_.features_.push_back(feature_to_add);
            // create a landmark
            // build the connection from landmark to feature
            Observation observation(frame_current_.keyframe_id_, feature_id);
            Landmark landmark_to_add(curr_landmark_id_, pts_3d.at(i), descriptors.row(i), reliable_depth.at(i), observation);
            curr_landmark_id_ ++;
            my_map_.insert_landmark(landmark_to_add);
            feature_id++;
        }
    }
    curr_keyframe_id_++;
    // insert the keyframe
    my_map_.insert_keyframe(frame_current_);
    return true;
}

void VO::move_frame()
{
    frame_last_ = frame_current_;
}

void VO::rviz_visualize()
{
    std::vector<cv::Point3f> pts_3d;
    pts_3d.clear();
    for(auto &lm : my_map_.landmarks_)
    {
        pts_3d.push_back(lm.second.pt_3d_);
    }
    my_visual_.publish_feature_map(pts_3d);
    my_visual_.publish_transform(T_c_w_);
    ros::spinOnce();
}

void VO::write_pose(const Frame &frame)
{
    SE3 T_w_c;
    T_w_c = frame.T_c_w_.inverse();
    double r00, r01, r02, r10, r11, r12, r20, r21, r22, x, y, z;
    Eigen::Matrix3d rotation = T_w_c.rotationMatrix();
    Eigen::Vector3d translation = T_w_c.translation();
    r00 = rotation(0, 0);
    r01 = rotation(0, 1);
    r02 = rotation(0, 2);
    r10 = rotation(1, 0);
    r11 = rotation(1, 1);
    r12 = rotation(1, 2);
    r20 = rotation(2, 0);
    r21 = rotation(2, 1);
    r22 = rotation(2, 2);
    x = translation(0);
    y = translation(1);
    z = translation(2);

    std::ofstream file;
    file.open("estimated_traj.txt", std::ios_base::app);
    file << frame.frame_id_ << " " << r00 << " " << r01 << " " << r02 << " " << x << " "
         << r10 << " " << r11 << " " << r12 << " " << y << " "
         << r20 << " " << r21 << " " << r22 << " " << z << std::endl;
    file.close();
}


};
