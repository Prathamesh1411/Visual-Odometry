#ifndef VOFRONTEND_H_
#define VOFRONTEND_H_

#include <iostream>
#include <cmath>
#include <include/library_include.hpp>
#include <vector>
#include <string>


using namespace std;
using namespace cv;

class VOFrontEnd
{
public:
    Frame frame_last_;
    Frame frame_current_;
    Map &my_map_;

    VOFrontEnd();
    ~VOFrontEnd();

    void extractFeatures(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
    void computeFeatureDescriptors(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    void keypointMatching();
    void triangulation();
    void motionEstimation();
};

#endif /*VOFRONTEND_H_*/
