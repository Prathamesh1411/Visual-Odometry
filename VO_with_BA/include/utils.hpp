#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"

using namespace std;

void getProjectionMatrices(string path);
vector<string> split (string s, string delimiter);
cv::Mat converttoHomogeneousMatrix(cv::Mat rotationMatrix, cv::Mat translationVector);

#endif /* utils_hpp */