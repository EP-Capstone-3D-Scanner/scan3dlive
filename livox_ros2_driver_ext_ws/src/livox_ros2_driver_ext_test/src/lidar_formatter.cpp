#include <memory>
#include <cmath> // Added for std::atan2, std::sqrt, and M_PI

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
      "livox/lidar", 10, std::bind(&LidarFormatter::topic_callback_lidar, this, _1));

      publisher_lidar_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar_formatted", 10);
      
      subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "livox/imu", 10, std::bind(&LidarFormatter::topic_callback_imu, this, _1));

      publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu_formatted", 10);
    }

  private:
    void topic_callback_lidar(const livox_interfaces_ext::msg::CustomMsg & msg_in) const
    {
      auto msg_out = livox_ros_driver2::msg::CustomMsg();

      msg_out.header = msg_in.header;
      msg_out.timebase = msg_in.timebase; 
      msg_out.lidar_id = msg_in.lidar_id;
      msg_out.rsvd = msg_in.rsvd;
      
      msg_out.points.reserve(msg_in.points.size());

      // Set the blind cone angle threshold (in degrees)
      const double angle_threshold_deg = 6.0;
      
      for (const auto& p : msg_in.points) {
          // Calculate the angle from the forward X-axis
          double distance_yz = std::sqrt(p.y * p.y + p.z * p.z);
          double angle_deg = std::atan2(distance_yz, p.x) * 180.0 / M_PI;

          // Skip this point if it falls within the central blind cone
          if (angle_deg < angle_threshold_deg) {
              continue;
          }

          uint8_t spatial_noise = p.tag & 0x03;
          uint8_t energy_noise = (p.tag >> 2) & 0x03;

          if ((spatial_noise == 1 || spatial_noise == 2 || spatial_noise == 3) || 
              (energy_noise == 1 || energy_noise == 2)) {
              continue; 
          }
          // -------------------------------------------------

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

      // CRITICAL: Update point_num to reflect the new size after filtering
      msg_out.point_num = msg_out.points.size();

      publisher_lidar_->publish(msg_out);
    }

    void topic_callback_imu(const sensor_msgs::msg::Imu & msg_in) const
    {
      auto msg_out = msg_in;
      publisher_imu_->publish(msg_out);
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