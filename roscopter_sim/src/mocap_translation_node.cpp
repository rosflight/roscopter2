#include <memory>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roscopter_msgs/msg/state.hpp"

#include "chrono"

using namespace std::chrono_literals;
using roscopter_msgs::msg::State;
using std::placeholders::_1;

class mocap_translation : public rclcpp::Node
{
public:
  mocap_translation() : Node("mocap_state")
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vrpn_mocap/JACOBflight/pose", qos_profile, std::bind(&mocap_translation::mocap_truth_callback, this, _1)); // z is east, x is north, y is up 
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&mocap_translation::imu_callback, this, _1)); // z is east, x is north, y is up 
    firmware_estimator_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("attitude/euler", 10, std::bind(&mocap_translation::firmware_estimator_callback, this, _1));

    publisher_ = this->create_publisher<roscopter_msgs::msg::State>("state", 10);
    // TODO: add GPS, baro and mag spoofed sensors.

    float frequency = 100.0;
    
    auto update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
    update_timer_ = this->create_wall_timer(update_period_, std::bind(&mocap_translation::publish_truth, this));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr firmware_estimator_sub_;

  rclcpp::Publisher<State>::SharedPtr publisher_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;

  void mocap_truth_callback(const geometry_msgs::msg::PoseStamped & msg)
  {
    mocap_truth_ = msg;
  }

  void firmware_estimator_callback(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    rpy_estimated_.vector.set__x(msg.vector.x);
    rpy_estimated_.vector.set__y(msg.vector.y);
    rpy_estimated_.vector.set__z(msg.vector.z);
  }

  //TODO insert wind callback.
  double wn = 0.0;
  double we = 0.0;
  double wd = 0.0;

  double v_n = 0.0;
  double v_e = 0.0;
  double v_d = 0.0;
  
  double p_n_prev = 0.0;
  double p_e_prev = 0.0;
  double p_d_prev = 0.0;

  geometry_msgs::msg::Vector3Stamped rpy_estimated_;
  geometry_msgs::msg::PoseStamped mocap_truth_;

  void publish_truth()
  {

    roscopter_msgs::msg::State state;

    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = 1; // Denotes global frame.

    state.initial_lat = 0.0;
    state.initial_lon = 0.0; // TODO implement correct initial lat and lon
    state.initial_alt = 0.0;

    state.position[0] = mocap_truth_.pose.position.x;
    state.position[1] = mocap_truth_.pose.position.z;
    state.position[2] = -mocap_truth_.pose.position.y;

    v_n = state.position[0] - p_n_prev; 
    v_n /= 2.0;
    v_e = state.position[1] - p_e_prev; 
    v_e /= 2.0;
    v_d = state.position[2] - p_d_prev; 
    v_d /= 2.0;

    p_n_prev = state.position[0];
    p_e_prev = state.position[1];
    p_d_prev = state.position[2];

    Eigen::Quaternionf q;
    q.w() = mocap_truth_.pose.orientation.w;
    q.x() = mocap_truth_.pose.orientation.x;
    q.y() = mocap_truth_.pose.orientation.z;
    q.z() = -mocap_truth_.pose.orientation.y;

    state.inclination = 0.0;

    Eigen::Vector3f euler;
    euler(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    euler(1) = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    euler(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    state.phi = rpy_estimated_.vector.x;
    state.theta = rpy_estimated_.vector.y;
    state.psi = rpy_estimated_.vector.z;

    // Eigen::Vector3f body_frame_velocity;
    // body_frame_velocity << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
    //
    // // Rotate the velocity vector from the body frame to the inertial frame.
    // Eigen::Matrix3f Rv_v1, Rv1_v2, Rv2_b;
    // Rv2_b << 1, 0, 0, 0, cos(state.phi), sin(state.phi), 0, -sin(state.phi), cos(state.phi);
    // Rv1_v2 << cos(state.theta), 0, -sin(state.theta), 0, 1, 0, sin(state.theta), 0, cos(state.theta);
    // Rv_v1 << cos(state.psi), sin(state.psi), 0, -sin(state.psi), cos(state.psi), 0, 0, 0, 1;
    //
    // Eigen::Matrix3f Rb_i = (Rv2_b * Rv1_v2 * Rv_v1).transpose();

    Eigen::Vector3f inertial_frame_velocity;
    inertial_frame_velocity << v_n, v_e, v_d;

    state.v_n = inertial_frame_velocity(0);
    state.v_e = inertial_frame_velocity(1);
    state.v_d = inertial_frame_velocity(2);

    state.vg = std::sqrt(pow(inertial_frame_velocity(0), 2)
		       + pow(inertial_frame_velocity(1), 2)
		       + pow(inertial_frame_velocity(2), 2));

    state.p = imu_.angular_velocity.x;
    state.q = imu_.angular_velocity.y;
    state.r = imu_.angular_velocity.z;

    state.quat_valid = true;

    state.quat[0] = mocap_truth_.pose.orientation.w;
    state.quat[1] = mocap_truth_.pose.orientation.x;
    state.quat[2] = mocap_truth_.pose.orientation.z;
    state.quat[3] = -mocap_truth_.pose.orientation.y;

    publisher_->publish(state);
  }

  sensor_msgs::msg::Imu imu_;

  void imu_callback(const sensor_msgs::msg::Imu & msg)
  {
    imu_.angular_velocity.x = msg.angular_velocity.x;
    imu_.angular_velocity.y = msg.angular_velocity.y;
    imu_.angular_velocity.z = msg.angular_velocity.z;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_translation>());
  rclcpp::shutdown();
  return 0;
}