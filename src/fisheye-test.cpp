#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "undistor.h"

std::string IMAGE_TOPIC = "/usb_fish_eye_front/image_raw";
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node {
 public:
    ImageSubscriber(): Node("image_subscriber") {
      _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, 10,std::bind(&ImageSubscriber::topic_callback, this, _1));

      _publisher = this->create_publisher<sensor_msgs::msg::Image>("undistort_img", 10);

      _static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      publishStaticTransform();
    }

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // cv::imshow("test", cv_ptr->image);

        Undistor _undistor;
        _undistor.ReciveSourceImg(cv_ptr);

        auto undistor_msg = cv_bridge::CvImage(msg->header,"bgr8",_undistor.undistort_img);
        _publisher->publish(*undistor_msg.toImageMsg());
        // cv::waitKey(1);

      } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }

    void publishStaticTransform() {
        geometry_msgs::msg::TransformStamped front_transform_stamped;
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
        _static_broadcaster->sendTransform(left_transform_stamped);

        geometry_msgs::msg::TransformStamped right_transform_stamped;
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
        _static_broadcaster->sendTransform(right_transform_stamped);

        geometry_msgs::msg::TransformStamped rear_transform_stamped;
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
        _static_broadcaster->sendTransform(rear_transform_stamped);

        // cam to imu
        geometry_msgs::msg::TransformStamped frontcam_transform_stamped;
        frontcam_transform_stamped.header.stamp             = this->get_clock()->now();
        frontcam_transform_stamped.header.frame_id            = "imu";
        frontcam_transform_stamped.child_frame_id             = "front_cam";
        frontcam_transform_stamped.transform.translation.x    = -0.01065;
        frontcam_transform_stamped.transform.translation.y    = -0.06309;
        frontcam_transform_stamped.transform.translation.z    = -0.11805;
        frontcam_transform_stamped.transform.rotation.w       = 0.560986;
        frontcam_transform_stamped.transform.rotation.x       = 0.430459;
        frontcam_transform_stamped.transform.rotation.y       = -0.430459;
        frontcam_transform_stamped.transform.rotation.z       = 0.560986;
        _static_broadcaster->sendTransform(frontcam_transform_stamped);

        geometry_msgs::msg::TransformStamped leftcam_transform_stamped;
        leftcam_transform_stamped.header.stamp                = this->get_clock()->now();
        leftcam_transform_stamped.header.frame_id             = "imu";
        leftcam_transform_stamped.child_frame_id              = "left_cam";
        leftcam_transform_stamped.transform.translation.x     = -0.24072;
        leftcam_transform_stamped.transform.translation.y     = 0.13982;
        leftcam_transform_stamped.transform.translation.z     = -0.19439;
        leftcam_transform_stamped.transform.rotation.w        = 0.560986;
        leftcam_transform_stamped.transform.rotation.x        = 0.430459;
        leftcam_transform_stamped.transform.rotation.y        = -0.430459;
        leftcam_transform_stamped.transform.rotation.z        = -0.560986;
        _static_broadcaster->sendTransform(leftcam_transform_stamped);

        geometry_msgs::msg::TransformStamped rightcam_transform_stamped;
        rightcam_transform_stamped.header.stamp               = this->get_clock()->now();
        rightcam_transform_stamped.header.frame_id            = "imu";
        rightcam_transform_stamped.child_frame_id             = "right_cam";
        rightcam_transform_stamped.transform.translation.x    = 0.24072;
        rightcam_transform_stamped.transform.translation.y    = 0.13983;
        rightcam_transform_stamped.transform.translation.z    = -0.19439;
        rightcam_transform_stamped.transform.rotation.w       = 0.560986;
        rightcam_transform_stamped.transform.rotation.x       = 0.430459;
        rightcam_transform_stamped.transform.rotation.y       = 0.430459;
        rightcam_transform_stamped.transform.rotation.z       = 0.560986;
        _static_broadcaster->sendTransform(rightcam_transform_stamped);

        geometry_msgs::msg::TransformStamped rearcam_transform_stamped;
        rearcam_transform_stamped.header.stamp                = this->get_clock()->now();
        rearcam_transform_stamped.header.frame_id             = "imu";
        rearcam_transform_stamped.child_frame_id              = "rear_cam";
        rearcam_transform_stamped.transform.translation.x     = -0.01045;
        rearcam_transform_stamped.transform.translation.y     = 0.49434;
        rearcam_transform_stamped.transform.translation.z     = -0.17166;
        rearcam_transform_stamped.transform.rotation.w        = 0.;
        rearcam_transform_stamped.transform.rotation.x        = 0. ;
        rearcam_transform_stamped.transform.rotation.y        = 0.608761;
        rearcam_transform_stamped.transform.rotation.z        = 0.793353;
        _static_broadcaster->sendTransform(rearcam_transform_stamped);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      _subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         _publisher;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>          _static_broadcaster;
};

int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;

  printf("hello world fisheye-undist package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();

  return 0;
}
