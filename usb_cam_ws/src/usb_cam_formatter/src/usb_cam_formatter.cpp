#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageCropper : public rclcpp::Node {
public:
  ImageCropper() : Node("usb_cam_formatter") {
    // Subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10, std::bind(&ImageCropper::image_callback, this, std::placeholders::_1));
    
    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image_cropped", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    try {
      // 1. USE SHARE: Point to the ROS message memory without copying. 
      // 2. REQUEST RGB8: Handle the color format immediately.
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "rgb8");

      // Define Region of Interest (ROI) for cropping the right half
      // Note: Taking an ROI in OpenCV does not copy data; it creates a new header.
      int half_width = cv_ptr->image.cols / 2;
      cv::Rect roi(half_width, 0, half_width, cv_ptr->image.rows);
      cv::Mat cropped_img = cv_ptr->image(roi);
      
      // Rotate the cropped image 180 degrees
      // This is the ONLY place where new memory is allocated for image pixels.
      cv::Mat rotated_img;
      cv::rotate(cropped_img, rotated_img, cv::ROTATE_180);
      
      // Convert to ROS Image message and publish via unique_ptr
      auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
      cv_bridge::CvImage(msg->header, "rgb8", rotated_img).toImageMsg(*out_msg);
      
      // Publish with std::move to enable ROS 2 zero-copy transport
      publisher_->publish(std::move(out_msg));

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