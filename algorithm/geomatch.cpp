#include "geomatch.h"

#include <opencv2/opencv.hpp>

#include <iostream>

#include <chrono>

namespace geomatch {

double deg2rad(double degree)
{
  return degree * M_PI / 180;
}

cv::Point2f rotate(cv::Point2f p, double th)
{
  cv::Point2f rp;
  rp.x = std::cos(th) * p.x - std::sin(th) * p.y;
  rp.y = std::sin(th) * p.x + std::cos(th) * p.y;
  return rp;
}

void geomatch_from_center(cv::Point2f temp_points[],int temp_points_num, cv::Mat search_img,cv::Point& result_pos, double& result_angle)
{
return; 
}

void geomatch(cv::Point2f temp_points[],int temp_points_num, cv::Mat search_img,cv::Point& result_pos, double& result_angle, double d_angle = 0.1, int angle_min = -1, int angle_max = -1, int x_min = -1, int x_max = -1, int y_min = -1, int y_max = -1)
{

  cv::Mat s_img = search_img;

  cv::Point2f *dot;
  dot =  new cv::Point2f[temp_points_num];

  double deg = 0;
  double deg_max_limit = 360;

  if (angle_min != -1)deg = angle_min;
  if (angle_max != -1)deg_max_limit = angle_max;

  int x_region_min = 0;
  int y_region_min = 0;
  int x_region_max = s_img.cols;
  int y_region_max = s_img.rows;

  std::cout << "*** search image size ***" << std::endl;
  std::cout << "w:" << x_region_max << std::endl;
  std::cout << "h:" << y_region_max << std::endl;

  if (x_min != -1)x_region_min = x_min;
  if (y_min != -1)y_region_min = y_min;
  if (x_max != -1)x_region_max = x_max;
  if (y_max != -1)y_region_max = y_max;

  std::cout << "*** region ***" << std::endl;
  std::cout << "x:" << x_min << " - " << x_max << std::endl;
  std::cout << "y:" << y_min << " - " << y_max << std::endl;

  int max_count = 0;
  cv::Point max_pos;
  double max_degree;

  double c1 = temp_points_num;
  double c2 = (deg_max_limit - deg) / d_angle;
  double c3 = x_region_max - x_region_min;
  double c4 = y_region_max - y_region_min;

/*
  std::cout << "temp points num:" << c1 << std::endl;
  std::cout << "angle num:" << c2 << std::endl;
  std::cout << "x num:" << c3 << std::endl;
  std::cout << "y num:" << c4 << std::endl;
*/
  std::cout << "calc cost:" << c1 * c2 * c3 * c4 / 1000000 << std::endl;

  while (deg <= deg_max_limit)
  {

    for (int i = 0; i < temp_points_num; i++)
    {
      dot[i] = rotate(temp_points[i], deg2rad(deg));
    }

    for (int j = y_region_min; j < y_region_max; j++)
    {
        for (int i = x_region_min; i < x_region_max; i++)
        {
          int counter = 0;
          for (int k=0;k<temp_points_num;k++)
           {
            int x = int(dot[k].x + i);
            int y = int(dot[k].y + j);
            if ((x > 0) && (y > 0) && (x < s_img.cols - 1) && ( y < s_img.rows - 1 ))
              if(s_img.at<unsigned char>(y,x) == 255)counter++;
           }
           if (counter > max_count)
           {
             max_count = counter;
             max_pos.x = i;
             max_pos.y = j;
             max_degree = deg;
           }
        }
    }
    deg += d_angle;
  }

  result_pos.x = max_pos.x;
  result_pos.y = max_pos.y;
  result_angle = max_degree;

  std::cout << "*** max pos ***" << std::endl;
  std::cout << "x:" << max_pos.x << std::endl;
  std::cout << "y:" << max_pos.y << std::endl;
  std::cout << "deg:" << max_degree << std::endl;
  std::cout << "count:" << max_count << std::endl;

  return;
}

cv::Mat write_points(cv::Point2f points[], int points_num, cv::Mat img, int center_x , int center_y  ,double degree)
{
  cv::Mat ret;
  cv::cvtColor(img, ret, cv::COLOR_GRAY2RGB);

  for (int i = 0; i < points_num; i++)
  {
    cv::Point2f p =  rotate(points[i], deg2rad(degree));
    int x = p.x + center_x; 
    int y = p.y + center_y; 


    if ((x > 0) && (y > 0) && (x < img.cols - 1) && ( y < img.rows - 1 ))
    {
      ret.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0);
    }
  }
  return ret;
}

cv::Mat pyrdown(int n, cv::Mat img)
{

  cv::Mat *imgs;
  imgs = new cv::Mat[n+1];
  imgs[0] = img;

  for(int i=0;i<n;i++)pyrDown(imgs[i], imgs[i+1], cv::Size(imgs[i].cols/2, imgs[i].rows/2));

  return imgs[n];
}

int func(cv::Mat template_img, cv::Mat search_img, int temp_canny_low, int temp_canny_high, int search_canny_low, int search_canny_high,cv::Point& result_pos, double& result_angle)
{
    cv::Mat origin = template_img.clone();
    cv::Mat simg = search_img.clone();

    if (origin.empty() || simg.empty())
        return 1;

    std::chrono::system_clock::time_point start, end;
    cv::Mat frame;

    cv::Canny(origin, frame, temp_canny_low, temp_canny_high);

    int x_sum = 0;
    int y_sum = 0;
    int width = frame.cols;
    int height = frame.rows;

    int noOfCordinates = 0; 
    cv::Point *coordinates;
    coordinates =  new cv::Point[width * height];   

    start = std::chrono::system_clock::now();

    for (int j = 0; j < frame.rows; j++)
    {
        unsigned char *src = frame.ptr<unsigned char>(j);
        for (int i = 0; i < frame.cols; i++)
        {
            if(src[i] == 255){
              coordinates[noOfCordinates].x = i;
              coordinates[noOfCordinates].y = j;
              x_sum += i;
              y_sum += j;
              noOfCordinates++;
            } 
        }
    }

    double center_x = double(x_sum) / noOfCordinates;
    double center_y = double(y_sum) / noOfCordinates;

    cv::Point2f *coordinates_from_center;
    coordinates_from_center =  new cv::Point2f[noOfCordinates];   

    for (int i=0;i<noOfCordinates;i++)
    {
      coordinates_from_center[i].x = coordinates[i].x - center_x;
      coordinates_from_center[i].y = coordinates[i].y - center_y;
    }

    int pyrdown_num = 4;

    double pyrdown_center_x = center_x / std::pow(2,pyrdown_num);
    double pyrdown_center_y = center_y / std::pow(2,pyrdown_num);

    cv::Mat dimg = pyrdown(pyrdown_num, origin);
    cv::Canny(dimg, dimg, temp_canny_low, temp_canny_high);
    int dimg_w = dimg.cols;
    int dimg_h = dimg.rows;
    int dot_num = 0; 
    cv::Point2f *dots;

    dots =  new cv::Point2f[dimg_w * dimg_h];   

    for (int j = 0; j < dimg.rows; j++)
    {
        unsigned char *src = dimg.ptr<unsigned char>(j);
        for (int i = 0; i < dimg.cols; i++)
        {
            if(src[i] == 255){
              circle(dimg, cv::Point(i, j), 1, 0, -1, cv::LINE_AA);
              src[i] = 255;
              dots[dot_num].x = (double)i - pyrdown_center_x;
              dots[dot_num].y = (double)j - pyrdown_center_y;
              dot_num++;
            } 
        }
    }

    int simg_w = simg.cols;
    int simg_h = simg.rows;

   cv::Mat dsimg = pyrdown(pyrdown_num, simg);
    cv::Canny(dsimg, dsimg, search_canny_low, search_canny_high);
    int dsimgW = dsimg.cols;
    int dsimgH = dsimg.rows;

    std::cout << "dots num:" << dot_num << std::endl;

    cv::Point max_pos;
    double max_degree;

    //void geomatch(cv::Point2f temp_points[],int temp_points_num, cv::Mat search_img,cv::Point& result_pos, double& result_angle, double d_angle = 0.1, int angle_min = -1, int angle_max = -1, int x_min = -1, int x_max = -1, int y_min = -1, int y_max = -1)
    geomatch(dots, dot_num, dsimg, max_pos, max_degree ,2);

    std::cout << "edge num:" << noOfCordinates << std::endl;
    std::cout << "image size:" << width << ", " << height << std::endl;
    std::cout << "center:" << center_x << ", " << center_y << std::endl;

    //imshow("frame", frame);
    //imshow("dimage", dimg);
    //imshow("dimage", dsimg);

    cv::Mat wframe = write_points( coordinates_from_center, noOfCordinates, simg, max_pos.x * std::pow(2, pyrdown_num), max_pos.y * std::pow(2, pyrdown_num), max_degree);


    double up_pos_x = max_pos.x * std::pow(2, pyrdown_num);
    double up_pos_y = max_pos.y * std::pow(2, pyrdown_num);

    double per = 0.1;

    int min_x = int(up_pos_x - width * per);
    int max_x = int(up_pos_x + width * per);
    int min_y = int(up_pos_y - height * per);
    int max_y = int(up_pos_y + height * per);
    int min_ang = int(max_degree - 360 * per);
    int max_ang = int(max_degree + 360 * per);

    if(min_x<0)min_x = 0;
    if(max_x>simg_w)min_x = simg_w;
    if(min_y<0)min_y = 0;
    if(max_y>simg_h)min_y = simg_h;

    cv::Canny(simg, simg, search_canny_low, search_canny_high);
    //void geomatch(cv::Point2f temp_points[],int temp_points_num, cv::Mat search_img,cv::Point& result_pos, double& result_angle, double d_angle = 0.1, int angle_min = -1, int angle_max = -1, int x_min = -1, int x_max = -1, int y_min = -1, int y_max = -1)
    std::cout << min_x << "," << max_x << "," << min_y << "," << max_y << std::endl;
    std::cout << min_ang << "," << max_ang << std::endl;

    geomatch(coordinates_from_center, noOfCordinates, simg, max_pos, max_degree , 2, min_ang, max_ang, min_x, max_x, min_y, max_y);
    //geomatch(coordinates_from_center, noOfCordinates, simg, max_pos, max_degree , 2, 359, 360);
    cv::Mat wframe2 = write_points( coordinates_from_center, noOfCordinates, simg, max_pos.x, max_pos.y, max_degree);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

    std::cout << "time:" << elapsed << std::endl;

//    imshow("w frame", wframe);
    cv::imshow("w frame2", wframe2);
//    imshow("simg", simg);
    //imwrite("w_frame.png", wframe);
    //waitKey(1);
    result_pos = max_pos;
    result_angle = max_degree;
    return 1;
}

}
