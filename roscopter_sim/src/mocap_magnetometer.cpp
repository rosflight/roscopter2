#include <cmath>
#include <memory>
#include <random>
#include "Eigen/Geometry"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include "Eigen/src/Core/Matrix.h"
#include "roscopter_sim/geomag.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <ctime>

#include "chrono"

#define EARTH_RADIUS 6378145.0f

using namespace std::chrono_literals;
using std::placeholders::_1;

class mocap_magnetometer : public rclcpp::Node
{
public:
  mocap_magnetometer() : Node("mocap_magnetometer")
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vrpn_mocap/ROScopter/pose", qos_profile, std::bind(&mocap_magnetometer::mocap_callback, this, _1)); // z is east, x is north, y is up

    publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/magnetometer", 10);

    frequency = 100.0; // TODO: make a param
    
    
    auto update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
    update_timer_ = this->create_wall_timer(update_period_, std::bind(&mocap_magnetometer::publish_mag, this));

    mag_stdev_ = 3000/1e9;

    k_mag_ = 7.0;

    init_lat_ = 40.245964;
    init_long_ = -111.647727;
    init_alt_ = 1387.0;

    load_magnetic_model();

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // Convert to tm struct for local time
    std::tm* local_time = std::localtime(&now_time);

    float decimal_month = local_time->tm_mon + local_time->tm_mday/31.0;
    float decimal_year = local_time->tm_year + 1900 + decimal_month/12.0;
    
    double total_intensity;
    double grid_variation;

    int mag_success = geomag_calc(init_alt_/1000.0,
                                  init_lat_,
                                  init_long_,
                                  decimal_year,
                                  &declination_,
                                  &inclination_,
                                  &total_intensity,
                                  &grid_variation);
    
    intertial_mag = Eigen::Vector3f::UnitX();

    declination_ *= M_PI / 180.0;
    inclination_ *= M_PI / 180.0;
    
    Eigen::Matrix3f mag_inclination_rotation;
    mag_inclination_rotation = Eigen::AngleAxisf(-inclination_, Eigen::Vector3f::UnitY());
    
    Eigen::Matrix3f mag_declination_rotation;
    mag_declination_rotation = Eigen::AngleAxisf(declination_, Eigen::Vector3f::UnitZ());
    
    // Find the magnetic field intenisty described in the inertial frame by rotating the frame.
    Eigen::Matrix3f mag_rotation = mag_declination_rotation*mag_inclination_rotation;

    inertial_mag_readings = mag_rotation*intertial_mag;
  }

private:
  double declination_;
  double inclination_;
  Eigen::Vector3f inertial_mag_readings;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;

  Eigen::Vector3d mag_gauss_markov_eta_;

  float k_mag_;
  float mag_stdev_;
  
  float frequency;

  Eigen::Quaternionf mocap_pose;

  void mocap_callback(const geometry_msgs::msg::PoseStamped & msg)
  {
    mocap_truth_ = msg;

    double T_s = 1/frequency;

    mocap_pose.w() = msg.pose.orientation.w;
    mocap_pose.x() = msg.pose.orientation.x;
    mocap_pose.y() = msg.pose.orientation.z;
    mocap_pose.z() = -msg.pose.orientation.y;

    std::random_device rd;
    std::mt19937 noise_generator(rd());
    std::normal_distribution<double> normal_distr(0.0, 1.0);
    
    Eigen::Vector3d mag_noise;
    mag_noise << mag_stdev_ * normal_distr(noise_generator),
                 mag_stdev_ * normal_distr(noise_generator),
                   mag_stdev_ * normal_distr(noise_generator);

    mag_gauss_markov_eta_ = std::exp(-k_mag_*T_s) * mag_gauss_markov_eta_ + T_s*mag_noise;
  }
  
  double init_lat_;
  double init_long_;
  double init_alt_;

  geometry_msgs::msg::PoseStamped mocap_truth_;

  sensor_msgs::msg::MagneticField mag_msg_;

  Eigen::Vector3f intertial_mag;

  void publish_mag()
  {
    std::random_device rd;
    std::mt19937 noise_generator(rd());
    std::normal_distribution<double> normal_distr(0.0, 1.0);

    float T_s = 1.0/frequency;

    // GazeboPose I_to_B = GZ_COMPAT_GET_WORLD_POSE(link_);
    auto R = mocap_pose.toRotationMatrix();

    Eigen::Vector3f current_mag_readings = R.transpose()*inertial_mag_readings / 50'000.0;

    // GazeboVector noise;
    // GZ_COMPAT_SET_X(noise, mag_stdev_ * normal_distribution_(noise_generator_));
    // GZ_COMPAT_SET_Y(noise, mag_stdev_ * normal_distribution_(noise_generator_));
    // GZ_COMPAT_SET_Z(noise, mag_stdev_ * normal_distribution_(noise_generator_));
    // mag_gauss_markov_eta_ = std::exp(-k_mag_*T_s) * mag_gauss_markov_eta_ + T_s*noise;


    mag_msg_.header.stamp = this->get_clock()->now();

    mag_msg_.magnetic_field.x = current_mag_readings(0);
    mag_msg_.magnetic_field.y = current_mag_readings(1);
    mag_msg_.magnetic_field.z = current_mag_readings(2);
    publisher_->publish(mag_msg_);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_magnetometer>());
  rclcpp::shutdown();
  return 0;
}
