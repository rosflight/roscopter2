#include "navigation/path_manager.hpp"

namespace roscopter 
{

// =========== UTILITY FUNCTIONS ===========
double dist(std::array<float, 3> array1, std::array<float, 3> array2)
{
  return sqrt(pow(array1[0] - array2[0], 2) + pow(array1[1] - array2[1], 2) + pow(array1[2] - array2[2], 2));
}

PathManager::PathManager() 
  : PathManagerROS()
  , current_wp_index_(0)
{
  // Hard code the waypoints for now
  roscopter_msgs::msg::Waypoint wp1;
  wp1.w = {0, 0, -40.0};
  wp1.speed = 0.0;
  wp1.psi = 0.0;
  waypoint_list_.push_back(wp1);

  roscopter_msgs::msg::Waypoint wp2;
  wp2.w = {0, 40.0, -40.0};
  wp2.speed = 5.0;
  wp2.psi = 0.0;
  waypoint_list_.push_back(wp2);

  roscopter_msgs::msg::Waypoint wp3;
  wp3.w = {40, 40.0, -40.0};
  wp3.speed = 5.0;
  wp3.psi = 0.0;
  waypoint_list_.push_back(wp3);

  // Initialize the path parameter sigma as in Algorithm 15
  current_wp_index_ = 1;
  previous_wp_index_ = 0;
  double s1 = waypoint_list_[previous_wp_index_].speed;
  sigma_(0) = 0;
  sigma_(1) = s1 / dist(waypoint_list_[current_wp_index_].w, waypoint_list_[previous_wp_index_].w);
}

void PathManager::declare_params()
{
  params.declare_double("default_altitude", 10.0);
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_path(double dt)
{
  roscopter_msgs::msg::TrajectoryCommand output_cmd;

  // Check if there are enough waypoints to follow
  if (waypoint_list_.size() < 1) {
    double default_altitude = params.get_double("default_altitude");

    roscopter_msgs::msg::TrajectoryCommand default_cmd;
    default_cmd.position[2] = -default_altitude;

    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints received! Orbiting the origin at " << std::to_string(default_altitude) << "!");

    return default_cmd;
  }

  // Otherwise, compute the control according to Algorithm 15 in Beard, McLain textbook
  // Update the path parameters
  if (sigma_(0) >= 1.0) {
    // TODO: Also check if we are close enough to the waypoint. If not, do a go-to-trajectory before resetting?
    previous_wp_index_ = current_wp_index_;
    current_wp_index_ = (current_wp_index_ + 1) % waypoint_list_.size();

    double s1 = waypoint_list_[previous_wp_index_].speed;
    sigma_(0) = 0;
    sigma_(1) = s1 / dist(waypoint_list_[current_wp_index_].w, waypoint_list_[previous_wp_index_].w);
  }

  // Integrate the path parameter
  rk4_step(dt);

  // Compute commanded trajectory
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];
  roscopter_msgs::msg::Waypoint prev_wp = waypoint_list_[previous_wp_index_];

  double s_i = curr_wp.speed;
  double s_i1 = prev_wp.speed;
  double norm = dist(curr_wp.w, prev_wp.w);
  double sigma_dot_1 = (pow(s_i, 2) + pow(s_i1, 2)) / (2 * pow(norm, 2));

  std::array<float, 3> difference;
  std::array<float, 3> position;
  std::array<float, 3> velocity;
  std::array<float, 3> acceleration;
  for (int i=0; i<3; ++i) {
    difference[i] = curr_wp.w[i] - prev_wp.w[i];

    position[i] = sigma_(0) * curr_wp.w[i] + (1 - sigma_(0)) * prev_wp.w[i];
    velocity[i] = sigma_(1) * difference[i];
    acceleration[i] = sigma_dot_1 * difference[i];
  }

  output_cmd.position = position;
  output_cmd.velocity = velocity;
  output_cmd.acceleration = acceleration;
  output_cmd.psi_cmd = sigma_(0) * curr_wp.psi + (1 - sigma_(0)) * prev_wp.psi;

  return output_cmd;
}

void PathManager::rk4_step(double dt)
{
    // TODO: add operator+ * to the struct
  Eigen::Vector2f F1 = F(sigma_);
  Eigen::Vector2f F2 = F(sigma_ + dt / 2 * F1);
  Eigen::Vector2f F3 = F(sigma_ + dt / 2 * F2);
  Eigen::Vector2f F4 = F(sigma_ + dt * F3);

  sigma_ = sigma_ + dt / 6 * (F1 + 2*F2 + 2*F3 + F4);
}

Eigen::Vector2f PathManager::F(Eigen::Vector2f sig)
{
  // Rename variables for clarity
  roscopter_msgs::msg::Waypoint current_wp = waypoint_list_[current_wp_index_];
  roscopter_msgs::msg::Waypoint previous_wp = waypoint_list_[previous_wp_index_];

  double s_i = current_wp.speed;
  double s_i1 = previous_wp.speed;
  double norm = dist(current_wp.w, previous_wp.w);

  Eigen::Vector2f sigma_dot;
  sigma_dot(0) = sig(1);
  sigma_dot(1) = (pow(s_i, 2) + pow(s_i1, 2)) / (2 * pow(norm, 2));

  return sigma_dot;
}

} // namespace roscopter