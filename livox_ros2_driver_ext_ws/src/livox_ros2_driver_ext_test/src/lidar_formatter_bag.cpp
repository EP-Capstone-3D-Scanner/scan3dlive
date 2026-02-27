#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "livox_interfaces_ext/msg/custom_point.hpp"
#include "livox_interfaces_ext/msg/custom_msg.hpp"

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class LidarFormatter : public rclcpp::Node
{
  public:
    LidarFormatter()
    : Node("lidar_formatter")
    {
      subscription_lidar_ = this->create_subscription<livox_interfaces_ext::msg::CustomMsg>(
      "livox/lidar", 20, std::bind(&LidarFormatter::topic_callback_lidar, this, _1));

      publisher_lidar_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar_formatted", 10);
      
      subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "livox/imu", 150, std::bind(&LidarFormatter::topic_callback_imu, this, _1));

      publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu_formatted", 10);
    }

  private:
    void topic_callback_lidar(const livox_interfaces_ext::msg::CustomMsg & msg_in) const
    {
      auto msg_out = livox_ros_driver2::msg::CustomMsg();

      auto current_time = this->now();

      msg_out.header = msg_in.header;
      // msg_out.header.stamp = current_time;
      msg_out.timebase = msg_in.timebase;
      msg_out.point_num = msg_in.point_num;
      msg_out.lidar_id = msg_in.lidar_id;
      msg_out.rsvd = msg_in.rsvd;
      
      msg_out.points.reserve(msg_in.points.size());
      
      for (const auto& p : msg_in.points) {
          livox_ros_driver2::msg::CustomPoint p_new;
          p_new.offset_time = p.offset_time;
          p_new.x = p.x;
          p_new.y = p.y;
          p_new.z = p.z;
          p_new.reflectivity = p.reflectivity;
          p_new.tag = p.tag;
          p_new.line = p.line;
          msg_out.points.push_back(p_new);
      }
      publisher_lidar_->publish(msg_out);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    void topic_callback_imu(const sensor_msgs::msg::Imu & msg_in) const
    {
      auto msg_out = msg_in;

      auto current_time = this->now();
      // msg_out.header.stamp = current_time;
      
      publisher_imu_->publish(msg_out);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<livox_interfaces_ext::msg::CustomMsg>::SharedPtr subscription_lidar_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_lidar_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFormatter>());
  rclcpp::shutdown();
  return 0;
}