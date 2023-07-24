#include "line_detect.h"
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

// std::this_thread::sleep_for(std::chrono::milliseconds(1000));

// open usb camera
// VideoCapture cap(0);

// open csi camera
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) 
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

    int capture_width = 640 ;
    int capture_height = 480 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 50 ;
    int flip_method = 2 ;

std::string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);

cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

imageProc::imageProc()
{
}

imageProc::~imageProc()
{
    cv::destroyAllWindows();
    // 关闭摄像头
    cap.release();
    cout << "Shutting down" << endl; 
}

bool imageProc::init(std::string yaml_file)
{
    // cap.set(CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 30);

    // 检查摄像头是否打开成功
    if (!cap.isOpened())
    {
        cout << "Failed to open camera!" << endl;
        return false;
    }
    YAML::Node yaml_node = YAML::LoadFile(yaml_file);
    LowerColor[0] = yaml_node["LowerColor0"].as<int>();
    LowerColor[1] = yaml_node["LowerColor1"].as<int>();
    LowerColor[2] = yaml_node["LowerColor2"].as<int>();
    UpperColor[0] = yaml_node["UpperColor0"].as<int>();
    UpperColor[1] = yaml_node["UpperColor1"].as<int>();
    UpperColor[2] = yaml_node["UpperColor2"].as<int>();
    width = yaml_node["Width"].as<int>();
    height = yaml_node["Height"].as<int>();
    crossTol = yaml_node["CrossTolerance"].as<int>();
    cHeight = yaml_node["CameraHeight"].as<double>();
    cTan = yaml_node["CameraTan"].as<double>();
    useAffine = yaml_node["UseAffine"].as<bool>();
    targetNum = yaml_node["TargetNum"].as<int>();
    if(useAffine)
    {
        transMatrixAffine = cv::Mat(3, 3, CV_64F);
        transMatrixAffine.at<double>(0,0) = yaml_node["Cam"]["TransMatrix"][0].as<double>();
        transMatrixAffine.at<double>(0,1) = yaml_node["Cam"]["TransMatrix"][1].as<double>();
        transMatrixAffine.at<double>(0,2) = yaml_node["Cam"]["TransMatrix"][2].as<double>();
        transMatrixAffine.at<double>(1,0) = yaml_node["Cam"]["TransMatrix"][3].as<double>();
        transMatrixAffine.at<double>(1,1) = yaml_node["Cam"]["TransMatrix"][4].as<double>();
        transMatrixAffine.at<double>(1,2) = yaml_node["Cam"]["TransMatrix"][5].as<double>();
        transMatrixAffine.at<double>(2,0) = yaml_node["Cam"]["TransMatrix"][6].as<double>();
        transMatrixAffine.at<double>(2,1) = yaml_node["Cam"]["TransMatrix"][7].as<double>();
        transMatrixAffine.at<double>(2,2) = yaml_node["Cam"]["TransMatrix"][8].as<double>();

    }
    cDegree = atan(cTan);
    for (int i = 0; i < 4; i++)
    {
        camIntrinsics[i] = yaml_node["Cam"]["intrinsics"][i].as<double>();
        camDist[i] = yaml_node["Cam"]["distortion"][i].as<double>();
    }


    cout << "camera is ready" << endl;
    std::thread mThread(&imageProc::process, this);
    mThread.detach();
    return true;
}

void imageProc::process()
{
    cv::Mat input, gray, map1, map2;
    // _decPoles.init("/home/jetson/WorkSpace/TrackingCar/src/robot_line/config/line.yaml");
    const cv::Mat K = (cv::Mat_<double>(3, 3) << camIntrinsics[0], 0, camIntrinsics[2],
                       0, camIntrinsics[1], camIntrinsics[3],
                       0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(4, 1) << camDist[0], camDist[1],
                       camDist[2], camDist[3]);
    cv::Size imageSize(width, height);
    const double alpha = 1; 
    
    while(true)
    {
        cap >> input;
        // cout << "Capturing FPS: " << cap.get(CAP_PROP_FPS) << endl;
        cv::Mat imgHsv, imgMask;
        std::vector<cv::Point2d> targetPoints;
        Eigen::Vector3d targetRealPoint;
        // transform BGR image to binary image through HSV
        cv::cvtColor(input,  imgHsv, cv::COLOR_BGR2HSV);
        cv::inRange(imgHsv, LowerColor, UpperColor, imgMask);
        cv::medianBlur(imgMask, imgMask, 5);
        int count = 0;
        cv::Point2f p;
        for(int i=0; i < targetNum; i++)
        {
            if (getTargetPoint(imgMask, targetPoints, i))
            {
                p.x =  static_cast<float>(targetPoints.back().x);
                p.y =  static_cast<float>(targetPoints.back().y);
                cv::circle(input, p, 5, cv::Scalar(255, 0, 0), -1);
                // targetRealPoint.push_back(getPointInReal_CameraModel(targetPoints.back()));
                count++;
            }
            
        }
        // if (count < 40)
        // {
        //     std::vector<std::pair<LS, LS>> lines = findLine(input);
        //     cv::Point2d p;
        //     for (int i = 0; i < lines.size(); i++)
        //     {
        //         std::cout << "hello result" << std::endl;
        //         p = lines[i].first.start;
                
        //     }
            
        //     std::cout << p << std::endl;
        // }
        
        cv::Vec4d targetLine;
        cv::Point2f pt1, pt2;
        fitLineRansac(targetPoints, targetLine);
        pt1.x = static_cast<float>(targetLine[2] - targetLine[0] / targetLine[1] * targetLine[3]);
        pt1.y = 0;
        pt2.x = static_cast<float>(targetLine[2] - targetLine[0] / targetLine[1] * (480 - targetLine[3]));
        pt2.y = 480;
        cv::line(input, pt1, pt2, cv::Scalar(0, 0, 255), 2);
        cv::Point2d finalTargetPoint(320,240);
        targetRealPoint = getPointInReal_CameraModel(finalTargetPoint);
        double theta =  cDegree;  // convert to radians
        double alpha = atan(targetRealPoint(1));
        std::cout << "cDegree : " << cDegree << " " << "theta : " << theta << " " << "alpha : " << alpha << std::endl;
        result.flag = 0;
        // cHeight : 相机高度 theta : 相机与竖直线的夹角 targetRealPoint : 通过camera_model得到的值 
        // alpha : 相机成像平面（x0,y0）坐标点的 atan(targetRealPoint(1)/1)
        // double factorK = (cHeight / tan(theta + alpha) + cHeight * tan(theta)) * cos(theta);
        // result.pointX = cHeight / (tan(theta - alpha));
        // result.pointY = factorK * targetRealPoint(1);
        // double factorK = (cHeight / cos(theta) -targetRealPoint(1)) / tan(theta);
        double factorK = (cHeight / tan(theta + alpha) + cHeight * tan(theta)) * cos(theta);
        result.pointX = -factorK * targetRealPoint(1) / sin(theta) + cHeight / tan(theta);
        result.pointY = -factorK * targetRealPoint(0);
        std::cout << "Z : " << factorK << " " << "X : " << result.pointX << " " << "Y : " << result.pointY << std::endl;
        std::cout << "X1 : " << cHeight / tan(theta + alpha) << " X2 : " << result.pointX << std::endl;
        std::cout << "targetReal : " << targetRealPoint(0) << " " << targetRealPoint(1) << std::endl;
        cv::Point p1(static_cast<int>(finalTargetPoint.x),static_cast<int>(finalTargetPoint.y));
        cv::circle(input, p1, 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("image", input);
        lineQue.push(result);
        waitKey(1);
    }
}

int imageProc::processCross(cv::Mat input, cv::Mat imgMask)
{
    int crossFlag = 0;
    std::vector<std::vector<cv::Point>> v;
    cv::Mat imgMaskTop = input;
    cv::Rect rect1, rect2;
    imgMaskTop(cv::Rect(0, 0.2*height, width, 0.8*height)) = 0;
    cv::findContours(imgMask, v, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (v.size() != 0) 
    {
        auto area = 0;
        uint32_t idx = 0;
        uint32_t count = 0;
        while (count < v.size()) 
        {
            if (area < v[count].size()) 
            {
                idx = count;
                area = v[count].size();
            }
            count++;
        }
        rect1 = boundingRect(v[idx]);
    }
    cv::findContours(imgMaskTop, v, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (v.size() != 0) 
    {
        auto area = 0;
        uint32_t idx = 0;
        uint32_t count = 0;
        while (count < v.size()) 
        {
            if (area < v[count].size()) 
            {
                idx = count;
                area = v[count].size();
            }
            count++;
        }
        rect2 = boundingRect(v[idx]);
    }
    if(rect1.width >= (width / 2 - crossTol))
    {
        if(rect1.width >= (width - crossTol * 2) && rect1.x <= (width / 2 -crossTol))
        {
            // "+" type cross
            if(rect2.y <= crossTol)
            {
                crossFlag = 1;
            }
            // "T" type cross
            else
            {
                crossFlag = 2;
            }
        }
        // "-|" type cross
        else if(rect1.x <= (width / 2 -crossTol))
        {
            crossFlag = 3; 
        }
        else if (rect1.x + rect1.width >= (width/2 + crossTol))
        {
            crossFlag = 4;
        }
    }
}

bool imageProc::getTargetPoint(cv::Mat input, std::vector<cv::Point2d> &targetP, int i)
{
    cv::Rect ROI(0, height / targetNum * i, width, height / targetNum);
    cv::Mat imgROI = input(ROI);
    cv::Moments M = cv::moments(imgROI);
    cv::Point2d p;
    if (M.m00 > 0) 
    {
        p.x = M.m10 / M.m00;
        p.y = M.m01 / M.m00 + static_cast<double>(height) / targetNum * i;
        targetP.push_back(p);
        return true;
    }
    else
    {
        return false;
    }
}

cv::Point2d imageProc::getPointInReal_AffineTrans(cv::Point2d p, cv::Mat &transMatrix)
{
	cv::Point2d resultPoint;
    resultPoint.x = (transMatrixAffine.at<double>(0, 0) * p.x + transMatrixAffine.at<double>(0, 1) * p.y + transMatrixAffine.at<double>(0, 2))/
    ((transMatrixAffine.at<double>(2, 0) * p.x + transMatrixAffine.at<double>(2, 1) * p.y + transMatrixAffine.at<double>(2, 2)));
    resultPoint.y = (transMatrixAffine.at<double>(1, 0) * p.x + transMatrixAffine.at<double>(1, 1) * p.y + transMatrixAffine.at<double>(1, 2))/
    ((transMatrixAffine.at<double>(2, 0) * p.x + transMatrixAffine.at<double>(2, 1) * p.y + transMatrixAffine.at<double>(2, 2)));
}

Eigen::Vector3d imageProc::getPointInReal_CameraModel(cv::Point2d p)
{
    camodocal::PinholeCamera cam;
    cam = camodocal::PinholeCamera("pinholeCam", width, height, camDist[0], camDist[1], camDist[2],
                                   camDist[3], camIntrinsics[0], camIntrinsics[1], camIntrinsics[2], camIntrinsics[3]);
    Eigen::Vector2d srcPoint;
    Eigen::Vector3d dstPoint;
    srcPoint << p.x, p.y;
    cam.liftProjective(srcPoint, dstPoint);
    return dstPoint;
    std::cout << "Point3D : " << dstPoint << std::endl;
}

std::vector<std::pair<LS, LS>> imageProc::findLine(cv::Mat input)
{
    DecPoles _decPoles;
    _decPoles.init("/home/jetson/WorkSpace/FindLine/src/robot_line/config/line.yaml");
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::pair<LS, LS>> pole_recs;
    pole_recs = _decPoles.detect_pole(gray, CV_8UC1);
    return pole_recs;
}

void imageProc::fitLineRansac(const std::vector<cv::Point2d>& points, cv::Vec4d &line, int iterations, double sigma, double k_min, double k_max)
{
    unsigned int n = points.size();
    cv::RNG rng;
    double bestScore = -1.;
    for(int k=0; k<iterations; k++)
    {
        int i1=0, i2=0;
        while(i1==i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2d& p1 = points[i1];
        const cv::Point2d& p2 = points[i2];
        // 直线的方向向量
        cv::Point2d dp = p2-p1;
        dp *= 1./norm(dp);
        double score = 0;

        if(dp.y/dp.x<=k_max && dp.y/dp.x>=k_min )
        {
            for(int i=0; i<n; i++)
            {
                cv::Point2d v = points[i]-p1;
                // 向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                double d = v.y*dp.x - v.x*dp.y;
                // score += exp(-0.5*d*d/(sigma*sigma));  //误差定义方式的一种
                if( fabs(d)<sigma )
                    score += 1;
            }
        }
        if(score > bestScore)
        {
            line = cv::Vec4d(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
}

void imageProc::fitLineLeastSquares(const std::vector<cv::Point2d>& points, cv::Vec4d &line)
{
    // 检查点的数量是否足够
    if(points.size() < 2)
    {
        std::cerr << "Error: Not enough points for fitting a line!" << std::endl;
        return;
    }

    // 计算点的平均值
    double xSum = 0.0, ySum = 0.0;
    for(const auto& point : points)
    {
        xSum += point.x;
        ySum += point.y;
    }
    double xMean = xSum / points.size();
    double yMean = ySum / points.size();

    // 计算最小二乘拟合
    double xxSum = 0.0, xySum = 0.0;
    for(const auto& point : points)
    {
        double dx = point.x - xMean;
        double dy = point.y - yMean;
        xxSum += dx * dx;
        xySum += dx * dy;
    }

    // 计算线的斜率和截距（假设线的形式为y = kx + b）
    double k = xySum / xxSum;
    double b = yMean - k * xMean;

    // 构建线参数，依据y = kx + b
    line = cv::Vec4d(k, -1.0, b, 0.0);
    // 如果你需要按照Ax + By + C = 0的格式返回线参数，可以将上面的代码替换为：
    // line = cv::Vec4d(-k, 1.0, -b, 0.0);

    std::cout << "Line fitting result: y = " << k << "x + " << b << std::endl;
}