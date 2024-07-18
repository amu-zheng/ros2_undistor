//
// Created by user on 24-7-2.
//

#ifndef UNDISTOR_H
#define UNDISTOR_H
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "tool.h"

class Undistor {
 public:
    Undistor() {
        K.at<float>(0,0) = 335.4000092418607;
        K.at<float>(0,2) = 411.1308628670982;
        K.at<float>(1,1) = 335.15418802126084;
        K.at<float>(1,2) = 313.24918559569375;
        D.at<float>(0,0) = -0.19626889117992818;
        D.at<float>(0,1) = 0.028515509016744323;
        D.at<float>(0,2) = -0.011383377343040752;
        D.at<float>(0,3) = 0.004353900021938105;

        _src_img = cv::Mat();
        undistort_img = cv::Mat();
        _time_stamp = 0;
    }

    void ReciveSourceImg(cv_bridge::CvImagePtr& img_ptr);

    cv::Mat undistort_img;

 private:
    void Processor();

    cv::Mat _src_img;
    cv::Mat K = cv::Mat::eye(3,3,CV_32F),
            D = cv::Mat::zeros(4,1,CV_32F);

    double _time_stamp;
};


#endif //UNDISTOR_H
