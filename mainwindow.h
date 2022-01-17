#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "./imagecropper/imagecropper.h"

#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_cropButton_clicked();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh_;
    ros::Subscriber string_sub_;
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    ImageCropper *w;
    QPixmap pixmap;
    cv::Mat template_img_;
};
#endif // MAINWINDOW_H

