#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "undistor.h"

using std::placeholders::_1;
class ImageSubscriber : public rclcpp::Node {
 public:
    ImageSubscriber(): Node("image_subscriber") {
      _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&ImageSubscriber::topic_callback, this, _1));
    }

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      cv::imshow("test", cv_ptr->image);
      Undistor _undistor;
      _undistor.ReciveSourceImg(cv_ptr);
      cv::waitKey(10);
    }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
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
