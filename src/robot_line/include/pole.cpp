#include "pole.h"
DecPoles::DecPoles()
{
}
void DecPoles::init(std::string yaml_file)
{
    YAML::Node yaml_node = YAML::LoadFile(yaml_file);
    minDis = yaml_node["MinDis"].as<double>();
    maxDis = yaml_node["MaxDis"].as<double>();
    kPole = yaml_node["kPole"].as<double>();
    LS_k = yaml_node["LS_k_1"].as<double>();
    minDisDiff = yaml_node["MinDisDiff"].as<double>();
    minPole = yaml_node["MinPole"].as<double>();
    _line_error = yaml_node["Line_error"].as<double>();
    _max_distance_between_two_lines = yaml_node["Max_distance_between_two_lines"].as<double>();
    _max_error = yaml_node["Max_error"].as<double>();
    _min_line_len = yaml_node["Min_line_len"].as<double>();
    // photo_string = yaml_node["Photo_string"].as<std::string>();
    ratio = yaml_node["Ratio"].as<double>();
    is_Line = yaml_node["Is_Line"].as<double>();
    aspect_ratio = yaml_node["Aspect_ratio"].as<double>();
    // 探测深度所需参数
    width = yaml_node["Width"].as<double>();
    height = yaml_node["Height"].as<double>();
}

double DecPoles::dis_two(LS &line_raw, cv::Point2d &point_)
{
    double distance;
    if (fabs(line_raw.start.x - line_raw.end.x) > 1)
    {
        double k = (line_raw.start.y - line_raw.end.y) / (line_raw.start.x - line_raw.end.x);
        double b = line_raw.start.y - k * line_raw.start.x;
        distance = fabs(point_.y - k * point_.x - b) / sqrt(1 + k * k);
    }
    else
    {
        distance = fabs(point_.x - line_raw.start.x);
    }
    return distance;
}

std::vector<LS> DecPoles::repair_Line(const std::vector<LS> &lines)
{
    std::vector<LS> tmp_Lines = lines;
    std::vector<LS> repaired_Lines_raw;
    std::vector<LS> repaired_Lines;
    // 默认上方为起点
    for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
    {
        if (i->start.y > i->end.y)
        {
            cv::Point2d t = i->end;
            i->end = i->start;
            i->start = t;
        }
        // 赋值k
        if (fabs(i->start.x - i->end.x) > 1)
        {
            i->k = (i->start.y - i->end.y) / (i->start.x - i->end.x);
        }
        else
        {
            i->k = 1024;
        }
    }
    for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
    {
        if (fabs(i->k) > kPole)
        {
            for (std::vector<LS>::iterator j = tmp_Lines.begin(); j != tmp_Lines.end() && i->start.y != -1; j++)
            {
                if (j->start.y != -1 && j != i && fabs(fabs(i->k) - fabs(j->k)) < 1 / LS_k && dis_two(*i, j->start) < is_Line && dis_two(*i, j->end) < is_Line)
                {
                    if (i->start.y > j->start.y)
                    {
                        i->start = j->start;
                    }
                    if (i->end.y < j->end.y)
                    {
                        i->end = j->end;
                    }
                    // 标志被检测过
                    j->start.y = -1;
                }
            }
            cv::Point i_point = i->start - i->end;
            if (i->start.y != -1 && i_point.x * i_point.x + i_point.y * i_point.y > minPole)
            {
                // std::cout << "i:" << i - tmp_Lines.begin() << " k:" << i->k;
                // std::cout << " i_length:" << i_point.x * i_point.x + i_point.y * i_point.y;
                // std::cout << " iend_x:" << i->end.x << " iend_y:" << i->end.y;
                // std::cout << " istart_x:" << i->start.x << " istart_y:" << i->start.y << std::endl;
                repaired_Lines_raw.push_back(*i);
            }
        }
    }
    for (std::vector<LS>::iterator i = repaired_Lines_raw.begin(); i != repaired_Lines_raw.end(); i++)
    {
        for (std::vector<LS>::iterator j = repaired_Lines_raw.begin(); j != repaired_Lines_raw.end() && i->start.y != -1; j++)
        {
            if (j->start.y != -1 && j != i && fabs(1 / fabs(i->k) - 1 / fabs(j->k)) < LS_k && dis_two(*i, j->start) < is_Line && dis_two(*i, j->end) < is_Line)
            {
                if (i->start.y > j->start.y)
                {
                    i->start = j->start;
                }
                if (i->end.y < j->end.y)
                {
                    i->end = j->end;
                }
                // 标志被检测过
                j->start.y = -1;
            }
        }
        if (i->start.y != -1)
        {
            repaired_Lines.push_back(*i);
        }
    }
    return repaired_Lines;
}

std::vector<std::pair<LS, LS>> DecPoles::match_line(const std::vector<LS> &lines)
{
    std::vector<std::pair<LS, LS>> pole;
    std::vector<LS> tmp_Lines = lines;
    std::vector<int> ismatch;
    ismatch.resize(lines.size(), 0);
    // std::cout << "***********" << std::endl;
    for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
    {
        if (ismatch[i - tmp_Lines.begin()] != 1)
        {
            cv::Point i_point = i->start - i->end;
            for (std::vector<LS>::iterator j = i + 1; j != tmp_Lines.end(); j++)
            {
                double ij_start = dis_two(*i, j->start);
                double ij_end = dis_two(*i, j->end);
                // std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
                //           << " ij_end:" << ij_end << " ij_start:" << ij_start << std::endl;
                if (ismatch[j - tmp_Lines.begin()] != 1 && fabs(ij_start - ij_end) < minDisDiff && fabs(ij_end) < maxDis && fabs(ij_end) > minDis)
                {
                    cv::Point j_point = j->start - j->end;
                    // std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
                    //           << " pole:" << i_point.x * i_point.x + i_point.y * i_point.y << std::endl;
                    double ratio_ = (double)(i_point.x * i_point.x + i_point.y * i_point.y) / (j_point.x * j_point.x + j_point.y * j_point.y);
                    // std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
                    //           << " pole:" << j_point.x * j_point.x + j_point.y * j_point.y;
                    // std::cout << " ratio:" << ratio_ << std::endl;
                    if (ratio_ < ratio && ratio_ > 1 / ratio)
                    {
                        pole.push_back(std::make_pair(*i, *j));
                        ismatch[j - tmp_Lines.begin()] = 1;
                        ismatch[i - tmp_Lines.begin()] = 1;
                    }
                }
            }
        }
    }
    return pole;
}

cv::Point2f DecPoles::FindFoot(const cv::Point2f &pntSart, const cv::Point2f &pntEnd, const cv::Point2f &pA)
{
    cv::Point2f pFoot;
    float k = 0.0;
    if (pntSart.x == pntEnd.x)
    {
        pFoot.x = pntSart.x;
        pFoot.y = pA.y;
        return pFoot;
    }
    k = (pntEnd.y - pntSart.y) * 1.0 / (pntEnd.x - pntSart.x);
    float A = k;
    float B = -1.0;
    float C = pntSart.y - k * pntSart.x;

    pFoot.x = (B * B * pA.x - A * B * pA.y - A * C) / (A * A + B * B);
    pFoot.y = (A * A * pA.y - A * B * pA.x - B * C) / (A * A + B * B);
    return pFoot;
}
// 根据得到的线段对得出杆子的矩形线段对
std::vector<std::pair<LS, LS>> DecPoles::pole_rec(const std::vector<std::pair<LS, LS>> &lines)
{
    std::vector<std::pair<LS, LS>> recs;
    for (int i = 0; i < lines.size(); i++)
    {
        std::pair<LS, LS> rec = lines[i];
        cv::Point2f start = FindFoot(lines[i].second.start, lines[i].second.end, lines[i].first.start);
        if (start.y < lines[i].second.start.y)
        {
            rec.second.start = start;
        }
        else
        {
            rec.first.start = FindFoot(lines[i].first.start, lines[i].first.end, lines[i].second.start);
        }
        cv::Point2f end = FindFoot(lines[i].second.start, lines[i].second.end, lines[i].first.end);
        if (end.y > lines[i].second.end.y)
        {
            rec.second.end = end;
        }
        else
        {
            rec.first.end = FindFoot(lines[i].first.start, lines[i].first.end, lines[i].second.end);
        }
        cv::Point2d width = rec.first.start - rec.second.start;
        cv::Point2d height = rec.first.start - rec.first.end;
        if (sqrt((height.x * height.x + height.y * height.y) / (width.x * width.x + width.y * width.y)) > aspect_ratio)
        {
            if (rec.first.start.x > rec.second.start.x)
            {
                LS t(rec.first.start, rec.first.end);
                // t = rec.first;
                rec.first = rec.second;
                rec.second = t;
            }
            recs.push_back(rec);
        }
    }
    return recs;
}

std::vector<std::pair<LS, LS>> DecPoles::detect_pole(cv::Mat &srcimg, int type)
{
    EDLines testED;
    if (type == CV_8UC1)
    {
        testED = EDLines(srcimg, _line_error, _min_line_len, _max_distance_between_two_lines, _max_error);
    }
    else if (type == CV_8UC3)
    {
        EDColor ab = EDColor(srcimg);
        testED = EDLines(ab, _line_error, _min_line_len, _max_distance_between_two_lines, _max_error);
    }
    std::vector<LS> lines = testED.getLines();
    std::vector<LS> rep = repair_Line(lines);
    std::vector<std::pair<LS, LS>> pole = match_line(rep);
    std::vector<std::pair<LS, LS>> pole_recs = pole_rec(pole);
    std::vector<std::pair<LS, LS>> pole_recs1;
    std::cout << pole_recs.size() << std::endl;
    for (int i = 0; i < pole_recs.size(); i++)
    {
        std::cout << "hello result" << std::endl;
        line(srcimg, pole_recs[i].first.start, pole_recs[i].first.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
        line(srcimg, pole_recs[i].second.start, pole_recs[i].second.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
        line(srcimg, pole_recs[i].second.start, pole_recs[i].first.start, cv::Scalar(0), 1, cv::LINE_AA, 0);
        line(srcimg, pole_recs[i].second.end, pole_recs[i].first.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
        // cv::imshow("result Image", srcimg);
        
    }

    return pole_recs;
}

void DecPoles::undiost_Init()
{
    const cv::Mat K = (cv::Mat_<double>(3, 3) << cam_intrinsics[0], 0, cam_intrinsics[2],
                       0, cam_intrinsics[1], cam_intrinsics[3],
                       0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(4, 1) << cam_dist[0], cam_dist[1],
                       cam_dist[2], cam_dist[3]);
    cv::Size imageSize(width, height);
    const double alpha = 1; //
    initUndistortRectifyMap(K, D, cv::Mat_<double>::eye(3, 3), K.clone(), imageSize, CV_16SC2, map1, map2);
}

