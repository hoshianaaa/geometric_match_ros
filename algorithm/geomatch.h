#ifndef GEOMATCH_H
#define GEOMATCH_H

#include <opencv2/opencv.hpp>

#include <iostream>

#include <chrono>

namespace geomatch {
double deg2rad(double);

cv::Point2f rotate(cv::Point2f, double);

void geomatch_from_center(cv::Point2f, int, cv::Mat, cv::Point&, double& );
void geomatch_from_center(cv::Point, int, cv::Mat, cv::Point&, double& );

void geomatch(cv::Point2f, int, cv::Mat, cv::Point&, double&, double&, double, int, int, int, int, int, int);

cv::Mat write_points(cv::Point2f points[], int points_num, cv::Mat img, int center_x = 0, int center_y = 0, double degree = 0);

cv::Mat pyrdown(int, cv::Mat);

int func(cv::Mat, cv::Mat, int, int, int, int, cv::Point& result_pos, double& result_angle);
}
#endif
