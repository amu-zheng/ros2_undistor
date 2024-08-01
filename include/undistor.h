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
enum CameraModel
{
    Mono,
    Stereo
};

class Undistor {
 public:
    Undistor() {
        K.at<float>(0,0) = 344.1486964886955;
        K.at<float>(0,2) = 344.8090099278455;
        K.at<float>(1,1) = 403.1471857237557;
        K.at<float>(1,2) = 294.0569464270307;
        D.at<float>(0,0) = -0.1907986481151707;
        D.at<float>(0,1) = 0.007115065151853113;
        D.at<float>(0,2) = 0.009078968907903051;
        D.at<float>(0,3) = -0.0028833409389138775;

        _src_img            = cv::Mat();
        _src_img1           = cv::Mat();
        undistort_img       = cv::Mat();
        undistort_img1      = cv::Mat();
        _time_stamp         = 0.;
    }

    void ReciveSourceImg(const cv_bridge::CvImagePtr& img_ptr);
    void RecivePairImg(cv_bridge::CvImagePtr& img0_ptr, cv_bridge::CvImagePtr& img1_ptr);

    cv::Mat undistort_img, undistort_img1;
    cv::Mat _src_img, _src_img1;

 private:
    void Processor();
    cv::Mat K = cv::Mat::eye(3,3,CV_32F),
            D = cv::Mat::zeros(4,1,CV_32F);

    double _time_stamp;
    CameraModel _cam_type = CameraModel::Mono;
};


#endif //UNDISTOR_H
