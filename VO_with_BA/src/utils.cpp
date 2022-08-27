#include "../include/utils.hpp"

using namespace std;

//Opens the calibration text file and returns the Left and Right Projection Matrices
void getProjectionMatrices(const std::string &path, cv::Mat &leftProjectionMatrix, cv::Mat &rightProjectionMatrix)
{
    ifstream calibFile;
    calibFile.open(path+"/calib.txt");
    if(calibFile.is_open()) {
        string line;
        getline(calibFile, line);
        vector<string> l1 = split(line, " ");

        l1.erase(l1.begin());

        for(int i = 0; i < 4; i++) {
            float dummy = stof(l1[i]);
            leftProjectionMatrix.at<float>(0,i) = dummy;
        }
        for(int i = 4; i < 8; i++) {
            float dummy = stof(l1[i]);
            leftProjectionMatrix.at<float>(1,i-4) = dummy;
        }
        for(int i = 8; i < 12; i++) {
            float dummy = stof(l1[i]);
            leftProjectionMatrix.at<float>(2,i-8) = dummy;
        }
        cout << "Left Projection Matrix: " << endl;
        cout << leftProjectionMatrix << endl;

        getline(calibFile, line);
        l1 = split(line, " ");
        l1.erase(l1.begin());
        for(int i = 0; i < 4; i++) {
            float dummy = stof(l1[i]);
            rightProjectionMatrix.at<float>(0,i) = dummy;
        }
        for(int i = 4; i < 8; i++) {
            float dummy = stof(l1[i]);
            rightProjectionMatrix.at<float>(1,i-4) = dummy;
        }
        for(int i = 8; i < 12; i++) {
            float dummy = stof(l1[i]);
            rightProjectionMatrix.at<float>(2,i-8) = dummy;
        }
        cout << "Right Projection Matrix: " << endl;
        cout << rightProjectionMatrix << endl;
        calibFile.close();
    } else {
        cout << "Calibration File Could Not Open" << endl;
    }
}

vector<string> split (string s, string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr (pos_start));
    return res;
}

cv::Mat converttoHomogeneousMatrix(cv::Mat rotationMatrix, cv::Mat translationVector)
{
    double zeros[3] = {0, 0, 0};
    cv::Mat zeros_mat = cv::Mat(1,3,CV_64F, zeros);
    cv::Mat ones_mat = cv::Mat(1,1,CV_64F, 1);
    cv::Mat temp1, temp2, temp3;
    vconcat(rotationMatrix, zeros_mat, temp1);
    vconcat(translationVector, ones_mat, temp2);
    hconcat(temp1, temp2, temp3);
    return temp3;
}