#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "livox_interfaces/msg/custom_point.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"

#include "livox_ros_driver2/msg/custom_msg.hpp"

using std::placeholders::_1;

class LidarFormatter : public rclcpp::Node
{
  public:
    LidarFormatter()
    : Node("lidar_formatter")
    {
      subscription_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
      "livox/lidar", 10, std::bind(&LidarFormatter::topic_callback, this, _1));

      publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar_formatted", 10);
    }

  private:
    void topic_callback(const livox_interfaces::msg::CustomMsg & msg_in) const
    {
      auto msg_out = livox_ros_driver2::msg::CustomMsg();

      auto current_time = this->now();

      // 1. Copy Metadata
      msg_out.header = msg_in.header;
      msg_out.header.stamp = current_time;
      msg_out.timebase = current_time.nanoseconds(); //msg_in.timebase;
      msg_out.point_num = msg_in.point_num;
      msg_out.lidar_id = msg_in.lidar_id;
      msg_out.rsvd = msg_in.rsvd;

      // 2. Efficiently transfer points
      // If you don't need msg_in anymore, you can't move from a SharedPtr const& 
      // easily, so we perform a vector assignment which is O(N).
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
      publisher_->publish(msg_out);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFormatter>());
  rclcpp::shutdown();
  return 0;
}