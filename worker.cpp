/*
   Copyright 2013 Fabien Pierre-Nicolas.
      - Primarily authored by Fabien Pierre-Nicolas

   This file is part of simple-qt-thread-example, a simple example to demonstrate how to use threads.
   This example is explained on http://fabienpn.wordpress.com/qt-thread-simple-and-stable-with-sources/

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This progra is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "worker.h"
#include <QTimer>
#include <QEventLoop>

#include <QThread>
#include <QDebug>

#include <iostream>

#include "./algorithm/geomatch.h"

Worker::Worker(QObject *parent) :
    QObject(parent)
{
    _working =false;
    _abort = false;

    result_num_ = 0;
    picking_pos_delta_ = cv::Point(0,0);

    this->search_canny_low_ = 50;
    this->search_canny_high_ = 100;

}

void Worker::requestWork()
{
    mutex.lock();
    _working = true;
    _abort = false;
    qDebug()<<"Request worker start in Thread "<<thread()->currentThreadId();
    mutex.unlock();

    emit workRequested();
}

void Worker::abort()
{
    mutex.lock();
    if (_working) {
        _abort = true;
        qDebug()<<"Request worker aborting in Thread "<<thread()->currentThreadId();
    }
    mutex.unlock();
}

void Worker::doWork()
{
    qDebug()<<"Starting worker process in Thread "<<thread()->currentThreadId();
    int i=0;
    while(1) {

        mutex.lock();
        bool abort = _abort;
        mutex.unlock();

        if (abort) {
            qDebug()<<"Aborting worker process in Thread "<<thread()->currentThreadId();
            break;
        }

        mutex.lock();
        cv::Mat t_img = this->template_img_;
        cv::Mat s_img = this->search_img_;
        int temp_canny_low = this->temp_canny_low_;
        int temp_canny_high = this->temp_canny_high_;
        int search_canny_low = this->search_canny_low_;
        int search_canny_high = this->search_canny_high_;
        mutex.unlock();

        cv::Point ret_p;
        double ret_ang;
        if(geomatch::func(t_img, s_img, temp_canny_low, temp_canny_high, search_canny_low, search_canny_high, ret_p, ret_ang)){

            //std::cout << "result angle1: " << ret_ang << std::endl;
            result_num_ = 1;
            result_pos_ = ret_p;
            result_angle_ = ret_ang;
        }else {
            result_num_ = 0;
        }

        //emit valueChanged(QString::number(i));
        i++;
    }

    // Set _working to false, meaning the process can't be aborted anymore.
    mutex.lock();
    _working = false;
    mutex.unlock();

    qDebug()<<"Worker process finished in Thread "<<thread()->currentThreadId();

    //Once 60 sec passed, the finished signal is sent
    emit finished();
}
