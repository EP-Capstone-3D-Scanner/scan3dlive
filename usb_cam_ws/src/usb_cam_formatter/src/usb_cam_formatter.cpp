#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageCropper : public rclcpp::Node {
public:
  ImageCropper()
  : Node("usb_cam_formatter")
  {
    // Subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10, std::bind(&ImageCropper::image_callback, this, std::placeholders::_1));
    
    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image_cropped", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    try {
      
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

      
      int half_width = cv_ptr->image.cols / 2;
      cv::Rect roi(half_width, 0, half_width, cv_ptr->image.rows);

      
      cv::Mat cropped_img = cv_ptr->image(roi);
      
      auto out_msg = cv_bridge::CvImage(msg->header, msg->encoding, cropped_img).toImageMsg();
      publisher_->publish(*out_msg);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCropper>());
  rclcpp::shutdown();
  return 0;
}