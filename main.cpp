#include "mainwindow.h"

#include <QApplication>
#include <ros/ros.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "geometric_match_ros");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    ros::Rate loop_rate(20);
    while (ros::ok()){
      ros::spinOnce();
      a.processEvents();
      loop_rate.sleep();
    }
}
