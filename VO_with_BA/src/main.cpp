#include <iostream>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "../include/vo_frontend.hpp"

using namespace std;

void processFrames(vector<string> leftImagesList, vector<string> rightImagesList)
{

    vector<cv::KeyPoint> prev_left_keypoints, prev_right_keypoints, cur_left_keypoints;
    cv::Mat prev_left_descriptors,prev_right_descriptors, cur_left_descriptors;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create(1000);

    for(int imageSeq = 1; imageSeq < leftImagesList.size(); imageSeq++)
    {
        cv::Mat prevLeftImage = cv::imread(leftImagesList[imageSeq - 1], cv::IMREAD_GRAYSCALE);
        cv::Mat prevRightImage = cv::imread(rightImagesList[imageSeq - 1], cv::IMREAD_GRAYSCALE);
        cv::Mat currLeftImage = cv::imread(leftImagesList[imageSeq], cv::IMREAD_GRAYSCALE);
        

    }
}

int main(int argc, char** argv)
{
    string sequence = argv[1];
    string path = "dataset/sequences/" + sequence;
    string leftImagesPath = path+"/image_0/*.png";
    string rightImagesPath = path+"/image_1/*.png";
    vector<string> leftImagesList;
    vector<string> rightImagesList;
    cv::glob(leftImagesPath, leftImagesList);
    cv::glob(rightImagesPath, rightImagesList);

    processFrames(leftImagesList, rightImagesList);
}



