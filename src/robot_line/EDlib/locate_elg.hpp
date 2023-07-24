#include "EDLib.h"
#include <opencv2/opencv.hpp>
int area = 1500;
mEllipse draw_elp(cv::Mat &in_Img, bool &isappeared)
{
    cv::Mat out_Img;
    cv::cvtColor(in_Img, out_Img, cv::COLOR_RGB2GRAY);
    EDPF testEDPF = EDPF(out_Img);
    EDCircles testEDCircles = EDCircles(testEDPF);

    // cv::Mat edgePFImage = testEDPF.getEdgeImage();
    // cv::imshow("Edge Image Parameter Free", edgePFImage);

    // Get circle information as [cx, cy, r]
    std::vector<mCircle> circles = testEDCircles.getCircles();
    // Get ellipse information as [cx, cy, a, b, theta]
    std::vector<mEllipse> ellipses = testEDCircles.getEllipses();

    int nocircle = circles.capacity();
    int noellipse = ellipses.capacity();
    int find_max_circle = 0;
    int find_max_ellipses = 0;
    if (nocircle != 0)
    {
        for (int i = 1; i < nocircle; i++)
        {
            if (circles[find_max_circle].r < circles[i].r)
                find_max_circle = i;
        }
    }
    if (noellipse != 0)
    {
        for (int i = 1; i < find_max_ellipses; i++)
        {
            if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
                find_max_ellipses = i;
        }
    }
    mEllipse c = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    if (noellipse != 0 && nocircle != 0)
    {
        if (pow(circles[find_max_circle].r, 2) > ellipses[find_max_ellipses].axes.area())
        {
            if (pow(circles[find_max_circle].r, 2) > area)
            {
                cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
                c = mEllipse(circles[find_max_circle].center, r, 0);
                isappeared = true;
            }
            else
            {
                isappeared = false;
            }
            // cv::circle(in_Img, circles[find_max_circle].center, (int)circles[find_max_circle].r, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        else
        {
            if (ellipses[find_max_ellipses].axes.area() > area)
            {
                isappeared = true;
                c = ellipses[find_max_ellipses];
            }
            else
            {
                isappeared = false;
            }
            // cv::ellipse(in_Img, ellipses[find_max_ellipses].center, ellipses[find_max_ellipses].axes, degree, 0.0, 360.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
    }
    if (nocircle != 0 && noellipse == 0)
    {
        if (pow(circles[find_max_circle].r, 2) > area)
        {
            cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
            c = mEllipse(circles[find_max_circle].center, r, 0);
            isappeared = true;
        }
        else
        {
            isappeared = false;
        }
        // cv::circle(in_Img, circles[find_max_circle].center, (int)circles[find_max_circle].r, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    if (nocircle == 0 && noellipse != 0)
    {
        if (ellipses[find_max_ellipses].axes.area() > area)
        {
            isappeared = true;
            c = ellipses[find_max_ellipses];
        }
        else
        {
            isappeared = false;
        }
        // cv::ellipse(in_Img, ellipses[find_max_ellipses].center, ellipses[find_max_ellipses].axes, degree, 0.0, 360.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    if (nocircle == 0 && noellipse == 0)
    {
        isappeared = false;
        c = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    }
    return c;
}