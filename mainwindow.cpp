#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "./imagecropper/imagecropper.h"
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->label->setText("Hello World!");
    string_sub_ = nh_.subscribe("image", 10, &MainWindow::callbackImage, this);

    QVBoxLayout *layout = new QVBoxLayout();
    w = new ImageCropper;
    layout->addWidget(w);
    ui->centralwidget->setLayout(layout);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {
    ui->label->setText("Sub!");
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat mat = cv_ptr->image;
    QImage image(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image);
    w->setImage(pixmap);
    //ui->label->setPixmap(pixmap.scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

