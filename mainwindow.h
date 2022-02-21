#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>

#include "./imagecropper/imagecropper.h"

#include <opencv2/opencv.hpp>

#include <QThread>
#include "worker.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void mousePressEvent(QMouseEvent *event); // add
    bool have_crop_img_;
    std::string file_path_;
    void set_template_image(cv::Mat mat, int canny_low=50, int canny_high=100);

private slots:
    void on_pushButton_clicked();
    void on_cropButton_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_horizontalSlider_2_valueChanged(int value);

    void on_horizontalSlider_3_valueChanged(int value);

    void on_horizontalSlider_4_valueChanged(int value);

    void on_doubleSpinBox_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh_;
    ros::Subscriber string_sub_;
    ros::Publisher result_pub_;
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    ImageCropper *w;
    QPixmap pixmap;
    cv::Mat template_img_, crop_image_;
    cv::Point2f *temp_dots_from_center_;

    int temp_dot_num_;
    int temp_canny_low_;
    int temp_canny_high_;
    int search_canny_low_;
    int search_canny_high_;

    double temp_dot_center_x_;
    double temp_dot_center_y_;

    double match_ratio_th_;

    QThread *thread;
    Worker *worker;
};
#endif // MAINWINDOW_H

