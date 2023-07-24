#include <opencv2/opencv.hpp>
#include <iostream>
#include "EDLib.h"
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>

class DecPoles
{
private:
    double minDis, maxDis;                                           // 判断两直线是否是一对
    double kPole;                                                    // 杆子斜率
    double LS_k;                                                     // 判断两线段是否为同一直线；
    double minDisDiff;                                               // 判断两线段是否为同一直线；
    double minPole;                                                  // 杆子长度最小值
    double _line_error, _max_distance_between_two_lines, _max_error; // EDlib检测所用
    int _min_line_len;                                               // EDlin检测线段最小长度
    double ratio;                                                    // 两根杆子的长度比值限制
    double is_Line;                                                  // 两根杆子的最小误差
    double aspect_ratio;                                             // 识别的长方形的长宽比限制

public:
    int width, height;                     // 图片长宽
    double cam_intrinsics[4], cam_dist[4]; // 前置相机内参
    cv::Mat map1, map2;                    // 相机去畸变矩阵
    // 默认初始参数
    DecPoles();
    // yaml读取参数
    void init(std::string yaml_file);
    // 计算点到直线的距离
    double dis_two(LS &line_raw, cv::Point2d &point_);
    // 修补直线
    std::vector<LS> repair_Line(const std::vector<LS> &lines);
    // 匹配两条杆子的直线
    std::vector<std::pair<LS, LS>> match_line(const std::vector<LS> &lines);
    // 求垂足
    cv::Point2f FindFoot(const cv::Point2f &pntSart, const cv::Point2f &pntEnd, const cv::Point2f &pA);
    // 根据得到的线段对得出杆子的矩形线段对
    std::vector<std::pair<LS, LS>> pole_rec(const std::vector<std::pair<LS, LS>> &lines);
    // 得出杆子
    std::vector<std::pair<LS, LS>> detect_pole(cv::Mat &srcimg, int type);
    // 图像去畸初始化
    void undiost_Init();

};
