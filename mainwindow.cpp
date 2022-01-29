#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "./imagecropper/imagecropper.h"
#include <QVBoxLayout>

#include <iostream>

#include <QDebug>

#include "./algorithm/geomatch.h"

#include <QMouseEvent> // add
#include <QtCore/QDebug> // add

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    string_sub_ = nh_.subscribe("image", 10, &MainWindow::callbackImage, this);

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
        //cv::drawMarker(mat, cv::Point(worker->result_pos_.x,worker->result_pos_.y), cv::Vec3b(200,0,0), cv::MARKER_CROSS);
    }else{
        cv::cvtColor(mat, mat,CV_GRAY2RGB);
    }

    QImage image2(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    QPixmap pixmap2 = QPixmap::fromImage(image2);
    ui->label_2->setPixmap(pixmap2.scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
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

    cv::cvtColor(mat, mat,CV_RGB2GRAY);
    template_img_ = mat;
    worker->setTemplateImage(mat);

    cv::Mat canny_temp;
    cv::Canny(mat, canny_temp, 50, 100);

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

        }

}
