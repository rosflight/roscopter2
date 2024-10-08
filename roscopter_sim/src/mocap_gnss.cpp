#include <cmath>
#include <memory>
#include <random>
#include "Eigen/Geometry"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosflight_msgs/msg/gnss_full.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chrono"

#define EARTH_RADIUS 6378145.0f

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
        "vrpn_mocap/ROScopter/pose", qos_profile, std::bind(&mocap_gnss::mocap_callback, this, _1)); // z is east, x is north, y is up

    publisher_ = this->create_publisher<rosflight_msgs::msg::GNSSFull>("gnss_full", 10);
    // TODO: add GPS, baro and mag spoofed sensors.

    frequency = 10.0; // TODO: make a param
    
    auto update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
    update_timer_ = this->create_wall_timer(update_period_, std::bind(&mocap_gnss::publish_gnss, this));

    gnss_full_msg_ = rosflight_msgs::msg::GNSSFull();

    horizontal_pos_stdev_ = 0.21;
    vertical_pos_stdev_ = 0.4;

    k_gnss_ = 1.0/1100;

    init_lat_ = 40.245964;
    init_long_ = -111.647727;
    init_alt_ = 1387.0;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;

  rclcpp::Publisher<rosflight_msgs::msg::GNSSFull>::SharedPtr publisher_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;

  Eigen::Vector3d gnss_gauss_markov_eta_;

  float k_gnss_;
  float horizontal_pos_stdev_;
  float vertical_pos_stdev_;
  
  float frequency;

  void mocap_callback(const geometry_msgs::msg::PoseStamped & msg) // TODO: create gnss data
  {
    mocap_truth_ = msg;

    double latitude;
    double longitude;
    double altitude;

    double T_s = 1/frequency;

    Eigen::Vector3d mocap_pose;
    mocap_pose << msg.pose.position.x, msg.pose.position.z, msg.pose.position.y; // NEU frame.
    
    mocap_pose += gnss_gauss_markov_eta_;

    std::random_device rd;
    std::mt19937 noise_generator(rd());
    std::normal_distribution<double> normal_distr(0.0, 1.0);
    
    Eigen::Vector3d pos_noise;
    pos_noise << horizontal_pos_stdev_ * normal_distr(noise_generator),
                 horizontal_pos_stdev_ * normal_distr(noise_generator),
                   vertical_pos_stdev_ * normal_distr(noise_generator);

    gnss_gauss_markov_eta_ = std::exp(-k_gnss_*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;

    gnss_full_msg_.lat = (mocap_pose(0) * 180.0 / M_PI / EARTH_RADIUS + init_lat_)*1e7;
    gnss_full_msg_.lon = (mocap_pose(1) * 180.0 / M_PI / (EARTH_RADIUS*cos(init_lat_ * M_PI/180.0)) + init_long_)*1e7;
    gnss_full_msg_.height = (mocap_pose(2) + init_alt_)*1e3;

    v_n += mocap_truth_.pose.position.x - p_n_prev;
    v_e += mocap_truth_.pose.position.z - p_e_prev;
    v_d += mocap_truth_.pose.position.y - p_d_prev;

    p_n_prev = mocap_truth_.pose.position.x;
    p_e_prev = mocap_truth_.pose.position.z;
    p_d_prev = mocap_truth_.pose.position.y;
  }
  
  double init_lat_;
  double init_long_;
  double init_alt_;

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
    gnss_full_msg_.vel_n = int(v_n / (1/frequency) * 1e3);
    gnss_full_msg_.vel_e = int(v_e / (1/frequency) * 1e3);
    gnss_full_msg_.vel_d = -int(v_d / (1/frequency) * 1e3);
    publisher_->publish(gnss_full_msg_);
    v_n = gnss_full_msg_.vel_n;
    p_n_prev = mocap_truth_.pose.position.x;
    p_e_prev = mocap_truth_.pose.position.z;
    p_d_prev = mocap_truth_.pose.position.y;
    v_n = 0.0;
    v_e = 0.0;
    v_d = 0.0;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_gnss>());
  rclcpp::shutdown();
  return 0;
}
