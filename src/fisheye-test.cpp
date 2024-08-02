#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <queue>
#include <mutex>
#include <thread>

#include "undistor.h"
#include "tool.h"

std::string IMAGE_TOPIC     = "/usb_fish_eye_front/image_raw";
std::string IMAGE0_TOPIC    = "/usb_cam0/image_raw";
std::string IMAGE1_TOPIC    = "/usb_cam1/image_raw";
double SYNC_TOLERANCE       = 0.003;

using std::placeholders::_1;
using std::placeholders::_2;

int NUM = 0;

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber(): Node("image_subscriber") {
        _publisher            = this->create_publisher<sensor_msgs::msg::Image>("cam0/image_raw", 10);
        _publisher_stereo     = this->create_publisher<sensor_msgs::msg::Image>("cam1/image_raw", 10);
        _static_broadcaster   = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        _undistor_ptr         = std::make_shared<Undistor>();
        // SuperTool::external_process();
        // publishStaticTransform();

        // Mono
        /*_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE0_TOPIC, 10,std::bind(&ImageSubscriber::topic_callback, this, _1));*/

        _image_subscriber0 = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE0_TOPIC, 10,std::bind(&ImageSubscriber::img0_callback, this, _1));
        _image_subscriber1 = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE1_TOPIC, 10,std::bind(&ImageSubscriber::img1_callback, this, _1));
    }

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
          const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          // cv::imshow("test", cv_ptr->image);
          _undistor_ptr->ReciveSourceImg(cv_ptr);
          const auto undistor_msg = cv_bridge::CvImage(msg->header,"bgr8", _undistor_ptr->undistort_img);
          _publisher->publish(*undistor_msg.toImageMsg());
          // cv::waitKey(1);

        } catch(cv_bridge::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
    }

    void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        _m_buf.lock();
        _img0_buf.push(img_msg);
        _m_buf.unlock();
    }

    void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        _m_buf.lock();
        _img1_buf.push(img_msg);
        _m_buf.unlock();
    }

    void sync_process() {
        while (true) {
            cv_bridge::CvImagePtr image0 = nullptr, image1 = nullptr;
            std_msgs::msg::Header header;
            _m_buf.lock();
            if (!_img0_buf.empty() && !_img1_buf.empty()) {
                double time0 = _img0_buf.front()->header.stamp.sec;
                double time1 = _img1_buf.front()->header.stamp.sec;
                // 0.003s sync tolerance
                if(time0 < time1 - SYNC_TOLERANCE) {
                    _img0_buf.pop();
                    LOG(INFO) << "throw img0";
                } else if(time0 > time1 + SYNC_TOLERANCE) {
                    _img1_buf.pop();
                    LOG(INFO) << "throw img1";
                } else {
                    LOG(INFO) << "receive tow images!!!!";
                    header = _img0_buf.front()->header;
                    image0 = cv_bridge::toCvCopy(_img0_buf.front(), sensor_msgs::image_encodings::BGR8);
                    _img0_buf.pop();
                    image1 = cv_bridge::toCvCopy(_img1_buf.front(), sensor_msgs::image_encodings::BGR8);
                    _img1_buf.pop();
                    // TODO(): chack chmod
                    char c = getchar();
                    if (c == 's')
                    {
                        std::string path = "/home/user/data/result/";
                        std::string path_1 = path + std::to_string(NUM) + "_left_" + std::to_string(image0->header.stamp.sec)
                                             + "_"+ std::to_string(image0->header.stamp.nanosec) + ".jpg";
                        std::string path_2 = path + std::to_string(NUM) + "_right_" + std::to_string(image1->header.stamp.sec)
                                             + "_"+ std::to_string(image1->header.stamp.nanosec) + ".jpg";
                        cv::imwrite(path_1, image0->image);
                        cv::imwrite(path_2, image1->image);
                        LOG(INFO) << "Save successfully!!!!";
                        NUM++;
                    }
                }
            }
            _m_buf.unlock();
            if(image0 != nullptr) {
                _undistor_ptr->RecivePairImg(image0, image1);
                const auto undistor_msg   = cv_bridge::CvImage(header,"bgr8", _undistor_ptr->undistort_img);
                const auto undistor_msg1  = cv_bridge::CvImage(header,"bgr8", _undistor_ptr->undistort_img1);
                _publisher->publish(*undistor_msg.toImageMsg());
                _publisher_stereo->publish(*undistor_msg1.toImageMsg());
            }
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void publishStaticTransform() {
      /*geometry_msgs::msg::TransformStamped front_transform_stamped;
      front_transform_stamped.header.stamp                = this->get_clock()->now();
      front_transform_stamped.header.frame_id               = "base_line";
      front_transform_stamped.child_frame_id                = "front_cam";
      front_transform_stamped.transform.translation.x       = 0.11015;
      front_transform_stamped.transform.translation.y       = 0.0211;
      front_transform_stamped.transform.translation.z       = 0.46245;
      front_transform_stamped.transform.rotation.w          = 0.608761;
      front_transform_stamped.transform.rotation.x          = 0.;
      front_transform_stamped.transform.rotation.y          = 0.793353;
      front_transform_stamped.transform.rotation.z          = 0.;
      _static_broadcaster->sendTransform(front_transform_stamped);

      geometry_msgs::msg::TransformStamped left_transform_stamped;
      left_transform_stamped.header.stamp                   = this->get_clock()->now();
      left_transform_stamped.header.frame_id                = "base_line";
      left_transform_stamped.child_frame_id                 = "left_cam";
      left_transform_stamped.transform.translation.x        = -0.09277;
      left_transform_stamped.transform.translation.y        = 0.25117;
      left_transform_stamped.transform.translation.z        = 0.53879;
      left_transform_stamped.transform.rotation.w           = 0.608761;
      left_transform_stamped.transform.rotation.x           = -0.793353;
      left_transform_stamped.transform.rotation.y           = 0.;
      left_transform_stamped.transform.rotation.z           = 0.;
      _static_broadcaster->sendTransform(left_transform_stamped);*/

      /*geometry_msgs::msg::TransformStamped right_transform_stamped;
      right_transform_stamped.header.stamp                  = this->get_clock()->now();
      right_transform_stamped.header.frame_id               = "base_line";
      right_transform_stamped.child_frame_id                = "right_cam";
      right_transform_stamped.transform.translation.x       = -0.09276;
      right_transform_stamped.transform.translation.y       = -0.23027;
      right_transform_stamped.transform.translation.z       = 0.53879;
      right_transform_stamped.transform.rotation.w          = 0.0;
      right_transform_stamped.transform.rotation.x          = 0.0;
      right_transform_stamped.transform.rotation.y          = 0.793353;
      right_transform_stamped.transform.rotation.z          = -0.608761;
      _static_broadcaster->sendTransform(right_transform_stamped);*/

      /*geometry_msgs::msg::TransformStamped rear_transform_stamped;
      rear_transform_stamped.header.stamp                 = this->get_clock()->now();
      rear_transform_stamped.header.frame_id                = "base_line";
      rear_transform_stamped.child_frame_id                 = "rear_cam";
      rear_transform_stamped.transform.translation.x        = -0.44728;
      rear_transform_stamped.transform.translation.y        = 0.0209;
      rear_transform_stamped.transform.translation.z        = 0.51606;
      rear_transform_stamped.transform.rotation.w           = -0.430459;
      rear_transform_stamped.transform.rotation.x           = 0.560986;
      rear_transform_stamped.transform.rotation.y           = 0.560986;
      rear_transform_stamped.transform.rotation.z           = -0.430459;
      _static_broadcaster->sendTransform(rear_transform_stamped);*/

      // cam to imu
      /*geometry_msgs::msg::TransformStamped frontcam_transform_stamped;
      frontcam_transform_stamped.header.stamp             = this->get_clock()->now();
      frontcam_transform_stamped.header.frame_id            = "imu";
      frontcam_transform_stamped.child_frame_id             = "front_cam";
      frontcam_transform_stamped.transform.translation.x    = -0.01065;
      frontcam_transform_stamped.transform.translation.y    = 0.06309;
      frontcam_transform_stamped.transform.translation.z    = 0.11805;
      frontcam_transform_stamped.transform.rotation.w       = -0.379928;
      frontcam_transform_stamped.transform.rotation.x       = 0.596368;
      frontcam_transform_stamped.transform.rotation.y       = -0.596368;
      frontcam_transform_stamped.transform.rotation.z       = -0.379928;
      _static_broadcaster->sendTransform(frontcam_transform_stamped);*/

      geometry_msgs::msg::TransformStamped leftcam_transform_stamped;
      leftcam_transform_stamped.header.stamp                = this->get_clock()->now();
      leftcam_transform_stamped.header.frame_id             = "imu";
      leftcam_transform_stamped.child_frame_id              = "down_cam";
      leftcam_transform_stamped.transform.translation.x     = 0.169668;
      leftcam_transform_stamped.transform.translation.y     = 0.00376189;
      leftcam_transform_stamped.transform.translation.z     = -0.0121544;
      leftcam_transform_stamped.transform.rotation.w        = 0.630404;
      leftcam_transform_stamped.transform.rotation.x        = -0.376417;
      leftcam_transform_stamped.transform.rotation.y        = 0.328413;
      leftcam_transform_stamped.transform.rotation.z        = -0.594176;
      _static_broadcaster->sendTransform(leftcam_transform_stamped);

      geometry_msgs::msg::TransformStamped rightcam_transform_stamped;
      rightcam_transform_stamped.header.stamp               = this->get_clock()->now();
      rightcam_transform_stamped.header.frame_id            = "imu";
      rightcam_transform_stamped.child_frame_id             = "up_cam";
      rightcam_transform_stamped.transform.translation.x    = 0.0996881;
      rightcam_transform_stamped.transform.translation.y    = 0.0068384;
      rightcam_transform_stamped.transform.translation.z    = 0.056891;
      rightcam_transform_stamped.transform.rotation.w       = 0.691532;
      rightcam_transform_stamped.transform.rotation.x       = -0.193411;
      rightcam_transform_stamped.transform.rotation.y       = 0.262262;
      rightcam_transform_stamped.transform.rotation.z       = -0.644666;
      _static_broadcaster->sendTransform(rightcam_transform_stamped);

      /*geometry_msgs::msg::TransformStamped rearcam_transform_stamped;
      rearcam_transform_stamped.header.stamp                = this->get_clock()->now();
      rearcam_transform_stamped.header.frame_id             = "imu";
      rearcam_transform_stamped.child_frame_id              = "rear_cam";
      rearcam_transform_stamped.transform.translation.x     = -0.01045;
      rearcam_transform_stamped.transform.translation.y     = -0.49434;
      rearcam_transform_stamped.transform.translation.z     = 0.17166;
      rearcam_transform_stamped.transform.rotation.w        = 0.;
      rearcam_transform_stamped.transform.rotation.x        = 0. ;
      rearcam_transform_stamped.transform.rotation.y        = 0.843391;
      rearcam_transform_stamped.transform.rotation.z        = -0.5373;
      _static_broadcaster->sendTransform(rearcam_transform_stamped);

      geometry_msgs::msg::TransformStamped imuscan_transform_stamped;
      imuscan_transform_stamped.header.stamp                = this->get_clock()->now();
      imuscan_transform_stamped.header.frame_id             = "imu";
      imuscan_transform_stamped.child_frame_id              = "base_line";
      imuscan_transform_stamped.transform.translation.x     = 0.04706;
      imuscan_transform_stamped.transform.translation.y     = 0.01045;
      imuscan_transform_stamped.transform.translation.z     = 0.3444;
      imuscan_transform_stamped.transform.rotation.w        = 0.707107;
      imuscan_transform_stamped.transform.rotation.x        = 0. ;
      imuscan_transform_stamped.transform.rotation.y        = 0.0;
      imuscan_transform_stamped.transform.rotation.z        = -0.707107;
      _static_broadcaster->sendTransform(imuscan_transform_stamped);*/

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      _subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      _image_subscriber0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      _image_subscriber1;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         _publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         _publisher_stereo;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>          _static_broadcaster;
    std::queue<sensor_msgs::msg::Image::SharedPtr>                _img0_buf;
    std::queue<sensor_msgs::msg::Image::SharedPtr>                _img1_buf;
    std::shared_ptr<Undistor>                                     _undistor_ptr;
    std::mutex                                                    _m_buf;
};

void log_init(const char* argv) {
    FLAGS_logbufsecs = 0;
    FLAGS_colorlogtostderr = true;
    // FLAGS_logtostderr = true;
    const std::string package_name = "fisheye-undist";
    const std::string log_path = ament_index_cpp::get_package_share_directory(package_name) + "/log";
    google::SetLogDestination(google::INFO, log_path.c_str());
    google::SetStderrLogging(google::INFO);
    google::InstallFailureSignalHandler();
    google::SetLogFilenameExtension(".log");
    google::InitGoogleLogging(argv);
}

int main(const int argc, char ** argv) {
    (void) argc;
    (void) argv;
    log_init(argv[0]);
    LOG(INFO) << "hello world fisheye-undist package";
    rclcpp::init(argc, argv);
    // TODO(amu): time synchronization
    auto node = std::make_shared<ImageSubscriber>();
    std::thread sync_thread{&ImageSubscriber::sync_process, node};
    rclcpp::spin(node);
    rclcpp::shutdown();
    google::ShutdownGoogleLogging();
    return 0;
}
