#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class LidarFormatter : public rclcpp::Node
{
  public:
    LidarFormatter()
    : Node("lidar_formatter")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", 10, std::bind(&LidarFormatter::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFormatter>());
  rclcpp::shutdown();
  return 0;
}