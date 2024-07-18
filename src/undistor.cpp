//
// Created by user on 24-7-2.
//

#include "undistor.h"

void Undistor::ReciveSourceImg(cv_bridge::CvImagePtr& img_ptr) {
    _src_img = img_ptr->image;
    _time_stamp = img_ptr->header.stamp.sec;
    Processor();
    SuperTool::external_process();
}

void Undistor::Processor() {
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K,
        cv::Size(_src_img.cols,_src_img.rows), CV_16SC2, map1, map2);
    cv::remap(_src_img, undistort_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // cv::imshow("undistort", undistort_img);
}