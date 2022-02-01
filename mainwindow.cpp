#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "./imagecropper/imagecropper.h"
#include <QVBoxLayout>

#include <iostream>

#include <QDebug>

#include "./algorithm/geomatch.h"

#include <QMouseEvent> // add
#include <QtCore/QDebug> // add

#include<string>
#include<fstream>
#include<iostream>

// ファイル存在確認: https://qiita.com/takahiro_itazuri/items/e999ae24ab34b2756b04
bool checkFileExistence(const std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //string_sub_ = nh_.subscribe("image", 10, &MainWindow::callbackImage, this);
    string_sub_ = nh_.subscribe("/usb_cam/image_raw", 10, &MainWindow::callbackImage, this);

    result_pub_ = nh_.advertise<geometry_msgs::Pose2D>("result", 1);

    w = new ImageCropper;
    ui->verticalLayout->addWidget(w);

    // thread関係
    thread = new QThread();
    worker = new Worker();

    worker->moveToThread(thread);
    //connect(worker, SIGNAL(valueChanged(QString)), ui->label, SLOT(setText(QString)));
    connect(worker, SIGNAL(workRequested()), thread, SLOT(start()));
    connect(thread, SIGNAL(started()), worker, SLOT(doWork()));
    connect(worker, SIGNAL(finished()), thread, SLOT(quit()), Qt::DirectConnection);

    worker->abort();
    thread->wait(); // If the thread is not running, this will immediately return.

    worker->requestWork();

    this->have_crop_img_ = 0;

    this->file_path_ = "/tmp/template.png";

    if(checkFileExistence(this->file_path_))
    {
        cv::Mat im = cv::imread(this->file_path_);
        this->set_template_image(im);
    }
}

MainWindow::~MainWindow()
{
    // thread 関係
    worker->abort();
    thread->wait();
    qDebug()<<"Deleting thread and worker in Thread "<<this->QObject::thread()->currentThreadId();
    delete thread;
    delete worker;

    delete ui;
}

void MainWindow::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat mat = cv_ptr->image;

    QImage image(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    pixmap = QPixmap::fromImage(image);

    cv::cvtColor(mat, mat,CV_RGB2GRAY);

    worker->setSearchImage(mat);

    //func(template_img_, mat, 50, 100, 50, 100);
    //cv::Canny(mat, mat, 50, 100);

    if(worker->result_num_){
        mat = geomatch::write_points(temp_dots_from_center_, temp_dot_num_, mat, worker->result_pos_.x, worker->result_pos_.y, worker->result_angle_);
        cv::drawMarker(mat, cv::Point(worker->result_pos_.x,worker->result_pos_.y), cv::Vec3b(150,150,150), cv::MARKER_CROSS);

        std::cout << "result angle:" << worker->result_angle_ << std::endl;

        cv::Point rotate_delta = geomatch::rotate(worker->picking_pos_delta_, geomatch::deg2rad(worker->result_angle_));

        cv::drawMarker(mat, cv::Point(worker->result_pos_.x + rotate_delta.x, worker->result_pos_.y + rotate_delta.y), cv::Vec3b(200,0,0), cv::MARKER_CROSS);

        //cv::drawMarker(mat, cv::Point(worker->result_pos_.x,worker->result_pos_.y), cv::Vec3b(200,0,0), cv::MARKER_CROSS);

        geometry_msgs::Pose2D msg;
        msg.x = worker->result_pos_.x + rotate_delta.x;
        msg.y = worker->result_pos_.y + rotate_delta.y;
        msg.theta = worker->result_angle_;
        this->result_pub_.publish(msg);

    }else{
        cv::cvtColor(mat, mat,CV_GRAY2RGB);
    }

    QImage image2(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    QPixmap pixmap2 = QPixmap::fromImage(image2);
    //ui->label_2->setPixmap(pixmap2.scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->label_2->setPixmap(pixmap2.scaled(400, 300, Qt::KeepAspectRatio, Qt::SmoothTransformation));

}


void MainWindow::on_pushButton_clicked()
{
    w->setImage(pixmap);
}

void MainWindow::on_cropButton_clicked()
{
    QPixmap pixmap = w->cropImage();
    QImage image(pixmap.toImage().convertToFormat(QImage::Format_RGB888));

    // qimage to cv::mat (参考) https://www.codetd.com/ja/article/6620060
    cv::Mat mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());

    this->set_template_image(mat);
    cv::imwrite(this->file_path_, mat);
}

void MainWindow::set_template_image(cv::Mat mat, int canny_low, int canny_high)
{

    cv::cvtColor(mat, mat,CV_RGB2GRAY);
    template_img_ = mat;
    worker->setTemplateImage(mat, canny_low, canny_high);

    cv::Mat canny_temp;
    cv::Canny(mat, canny_temp, canny_low, canny_high);

    // template imageのエッジ座標取得
    int tempW = canny_temp.cols;
    int tempH = canny_temp.rows;
    temp_dot_num_ = 0;
    cv::Point2f *temp_dots;
    temp_dots = new cv::Point2f[tempW * tempH];

    int temp_dot_x_sum = 0;
    int temp_dot_y_sum = 0;

    for(int j=0;j<tempH;j++){
        for(int i=0;i<tempW;i++){
            if(canny_temp.at<unsigned char>(j,i) == 255){
                temp_dots[temp_dot_num_].x = i;
                temp_dots[temp_dot_num_].y = j;
                temp_dot_x_sum += i;
                temp_dot_y_sum += j;
                temp_dot_num_++;
            }
        }
    }

    temp_dot_center_x_ = double(temp_dot_x_sum) / temp_dot_num_;
    temp_dot_center_y_ = double(temp_dot_y_sum) / temp_dot_num_;

    temp_dots_from_center_ = new cv::Point2f[temp_dot_num_];

    for(int i=0;i<temp_dot_num_;i++)
    {
        temp_dots_from_center_[i].x = temp_dots[i].x - temp_dot_center_x_;
        temp_dots_from_center_[i].y = temp_dots[i].y - temp_dot_center_y_;
    }
    // エッジ取得ここまで

    mat = geomatch::write_points(temp_dots_from_center_, temp_dot_num_, mat, temp_dot_center_x_, temp_dot_center_y_);
    cv::drawMarker(mat, cv::Point(temp_dot_center_x_,temp_dot_center_y_), cv::Vec3b(150,150,150), cv::MARKER_CROSS);

    worker->picking_pos_delta_.x = 0;
    worker->picking_pos_delta_.y = 0;

    this->crop_image_ = mat.clone();

    cv::drawMarker(mat, cv::Point(temp_dot_center_x_,temp_dot_center_y_), cv::Vec3b(200,0,0), cv::MARKER_CROSS);

    //cv::cvtColor(mat, mat,CV_GRAY2RGB);

    QImage image2(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    pixmap = QPixmap::fromImage(image2);

    ui->label->setPixmap(pixmap);

    this->have_crop_img_ = 1;

}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
        // get mouse pos:
        // 参考: https://www.qtcentre.org/threads/3073-How-to-get-mouse-s-position
        QPoint p = ui->label->mapFromGlobal(QCursor::pos());
        qDebug() << "press:" << p;

        if(this->have_crop_img_)
        {
            worker->picking_pos_delta_.x = p.x() - temp_dot_center_x_;
            worker->picking_pos_delta_.y = p.y() - temp_dot_center_y_;

            cv::Mat mat = this->crop_image_.clone();

            cv::drawMarker(mat, cv::Point(temp_dot_center_x_ + worker->picking_pos_delta_.x,temp_dot_center_y_ + worker->picking_pos_delta_.y), cv::Vec3b(200,0,0), cv::MARKER_CROSS);

            //cv::cvtColor(mat, mat,CV_GRAY2RGB);

            QImage image2(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
            pixmap = QPixmap::fromImage(image2);

            ui->label->setPixmap(pixmap);

        }
}
