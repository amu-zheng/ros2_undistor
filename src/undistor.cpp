//
// Created by user on 24-7-2.
//

#include "undistor.h"

void Undistor::ReciveSourceImg(const cv_bridge::CvImagePtr& img_ptr) {
    _src_img            = img_ptr->image;
    _time_stamp         = img_ptr->header.stamp.sec;
    Processor();
}

void Undistor::RecivePairImg(cv_bridge::CvImagePtr& img0_ptr, cv_bridge::CvImagePtr& img1_ptr)
{
    // LOG(WARNING) << "recive two images!!!";
    _src_img            = img0_ptr->image;
    _src_img1           = img1_ptr->image;
    _time_stamp         = img0_ptr->header.stamp.sec;
    _cam_type           = CameraModel::Stereo;
    Processor();
}


void Undistor::Processor() {
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K,
        cv::Size(_src_img.cols,_src_img.rows), CV_16SC2, map1, map2);
    cv::remap(_src_img, undistort_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // cv::imshow("undistort", undistort_img);

    if (_cam_type == CameraModel::Stereo)
    {
        cv::Mat K1 = cv::Mat::eye(3,3,CV_32F),
        D1 = cv::Mat::zeros(4,1,CV_32F);
        K1.at<float>(0,0) = 345.61212793323824;
        K1.at<float>(0,2) = 345.78938902784006;
        K1.at<float>(1,1) = 423.2759750430963;
        K1.at<float>(1,2) = 308.5126498575567;
        D1.at<float>(0,0) = -0.1942206532538713;
        D1.at<float>(0,1) = 0.004710486245556949;
        D1.at<float>(0,2) = 0.01768514200939357;
        D1.at<float>(0,3) = -0.007237334493056394;
        cv::fisheye::initUndistortRectifyMap(K1, D1, cv::Mat(), K1,
            cv::Size(_src_img1.cols,_src_img1.rows), CV_16SC2, map1, map2);
        cv::remap(_src_img1, undistort_img1, map1, map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        // cv::imshow("test", undistort_img1);
    }
    // cv::waitKey(1);
}