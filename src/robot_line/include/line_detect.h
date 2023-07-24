#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

#include <iostream>
#include <string>
#include <queue>
#include <thread>
#include <fstream>
#include <cmath>
#include <boost/format.hpp> 
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>  // cv::Canny()
#include <yaml-cpp/yaml.h>

#include "pole.h"
#include "CataCamera.h"
#include "PinholeCamera.h"


struct lineData
{
    int flag;
    double pointX;
    double pointY;
};


class imageProc
{
public:
    imageProc();
    ~imageProc();
    bool init(std::string yaml_file);

    cv::Scalar LowerColor = {0, 0, 0};                  
    cv::Scalar UpperColor = {0, 0, 0};
    bool useAffine;
    int width = 0;
    int height = 0;
    int crossTol = 0;
    double cHeight = 0;
    double cTan = 0;
    double cDegree = 0;
    double cDepth = 0;
    double camIntrinsics[4], camDist[4]; 
    // the number of points generated in track
    int targetNum;

    // Affine transform Matrix
    cv::Mat transMatrixAffine;

    // transform pixels in image to pixels in reality
    Eigen::Vector3f cameraWorld;
    Eigen::Vector3f realityWorld;
    Eigen::Matrix3f factorK;

    // DecPoles _decPoles;
    // transformed data
    lineData result;
    // store transformed data 
    std::queue<lineData> lineQue;
    // imgae processing for central line detection
    void process(); 
    // image processing for cross detection
    int processCross(cv::Mat input, cv::Mat imgMask);
    // find next target point by calculating the geometric moment
    bool getTargetPoint(cv::Mat input, std::vector<cv::Point2d> &targetP, int i);
    // get point's position in real world by affine transformation
    cv::Point2d getPointInReal_AffineTrans(cv::Point2d p, cv::Mat &transMatrix);
    // get point's position in real world by camera model
    Eigen::Vector3d getPointInReal_CameraModel(cv::Point2d p);
    // use edlib to find lines in image
    std::vector<std::pair<LS, LS>> findLine(cv::Mat input);
    // fit line by RanSAC
    void fitLineRansac(const std::vector<cv::Point2d>& points, cv::Vec4d &line, int iterations = 2000, double sigma = 1, double k_min = -7., double k_max = 7.);
    void fitLineLeastSquares(const std::vector<cv::Point2d>& points, cv::Vec4d &line);
};

#endif