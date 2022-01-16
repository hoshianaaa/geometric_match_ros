#include "mainwindow.h"

#include <QApplication>
#include <ros/ros.h> // 追加


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "geometric_match_ros"); // 追加

    ros::NodeHandle nh = ros::NodeHandle(); // 追加

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

