#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "./imagecropper/imagecropper.h"
#include <QVBoxLayout>

#include <iostream>

#include "./algorithm/geomatch.h"

#include <QDebug>

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
    connect(worker, SIGNAL(valueChanged(QString)), ui->label, SLOT(setText(QString)));
    connect(worker, SIGNAL(workRequested()), thread, SLOT(start()));
    connect(thread, SIGNAL(started()), worker, SLOT(doWork()));
    connect(worker, SIGNAL(finished()), thread, SLOT(quit()), Qt::DirectConnection);

    worker->abort();
    thread->wait(); // If the thread is not running, this will immediately return.

    worker->requestWork();


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

    func(template_img_, mat, 50, 100, 50, 100);
    //cv::Canny(mat, mat, 50, 100);
    //cv::cvtColor(mat, mat,CV_GRAY2RGB);

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
    //cv::Canny(mat, mat, 50, 100);
    //cv::cvtColor(mat, mat,CV_GRAY2RGB);

    QImage image2(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
    pixmap = QPixmap::fromImage(image2);

    ui->label->setPixmap(pixmap);
}
