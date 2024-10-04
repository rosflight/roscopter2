#include <memory>
#include "Eigen/Geometry"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosflight_msgs/msg/gnss_full.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chrono"

using namespace std::chrono_literals;
using std::placeholders::_1;

class mocap_gnss : public rclcpp::Node
{
public:
  mocap_gnss() : Node("gnss_full_mocap")
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vrpn_mocap/JACOBflight/pose", qos_profile, std::bind(&mocap_gnss::mocap_callback, this, _1)); // z is east, x is north, y is up

    publisher_ = this->create_publisher<rosflight_msgs::msg::GNSSFull>("gnss_full", 10);
    // TODO: add GPS, baro and mag spoofed sensors.

    float frequency = 10.0; // TODO: make a param
    
    auto update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
    update_timer_ = this->create_wall_timer(update_period_, std::bind(&mocap_gnss::publish_gnss, this));

    gnss_full_msg_ = rosflight_msgs::msg::GNSSFull();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;

  rclcpp::Publisher<rosflight_msgs::msg::GNSSFull>::SharedPtr publisher_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;

  void mocap_callback(const geometry_msgs::msg::PoseStamped & msg) // TODO: create gnss data
  {
    mocap_truth_ = msg;
  }

  double v_n = 0.0;
  double v_e = 0.0;
  double v_d = 0.0;
  
  double p_n_prev = 0.0;
  double p_e_prev = 0.0;
  double p_d_prev = 0.0;

  geometry_msgs::msg::PoseStamped mocap_truth_;

  rosflight_msgs::msg::GNSSFull gnss_full_msg_;

  void publish_gnss()
  {
    publisher_->publish(gnss_full_msg_);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_gnss>());
  rclcpp::shutdown();
  return 0;
}
