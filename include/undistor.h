//
// Created by user on 24-7-2.
//

#ifndef UNDISTOR_H
#define UNDISTOR_H
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class Undistor {
 public:
    Undistor() {
        K.at<float>(0,0) = 438.783367;
        K.at<float>(0,2) = 305.593336;
        K.at<float>(1,1) = 437.302876;
        K.at<float>(1,2) = 243.738352;
        D.at<float>(0,0) = -0.361976;
        D.at<float>(0,1) = 0.11051;
        D.at<float>(0,2) = 0.001014;
        D.at<float>(0,3) = 0.000505;

        _src_img = cv::Mat();
        undistort_img = cv::Mat();
        _time_stamp = 0;
    }

    void ReciveSourceImg(cv_bridge::CvImagePtr& img_ptr);

 private:
    void Processor();


    cv::Mat _src_img;
    cv::Mat undistort_img;
    cv::Mat K = cv::Mat::eye(3,3,CV_32F),
            D = cv::Mat::zeros(4,1,CV_32F);

    double _time_stamp;

};



#endif //UNDISTOR_H
