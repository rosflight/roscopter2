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
  // Initialize simga_0 to infinity
  sigma_(0) = INFINITY;

  declare_params();
}

void PathManager::declare_params()
{
  params.declare_double("default_altitude", 10.0);
  params.declare_bool("hover_last", 0);
  params.declare_double("waypoint_tolerance", 10.0);
  params.declare_double("path_update_dt", 0.005);
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_path()
{
  roscopter_msgs::msg::TrajectoryCommand output_cmd;

  // Check the number of waypoints - do we have enough to do path management?
  if (waypoint_list_.size() < 1) {
    // If there are no waypoints added, hover at the origin at the default altitude
    double default_altitude = params.get_double("default_altitude");
    output_cmd.position[2] = -default_altitude;

    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints received! Orbiting the origin at " << std::to_string(default_altitude) << "!");
    return output_cmd;
  }
  else if (waypoint_list_.size() == 1) {
    // If only one waypoint is added, do a go-to-point trajectory
    roscopter_msgs::msg::Waypoint wp = waypoint_list_[0];
    output_cmd.position = wp.w;
    output_cmd.psi_cmd = wp.psi;

    return output_cmd;
  }

  // Check to see if we are close enough to the first waypoint before continuing
  // double waypoint_tolerance = params.get_double("waypoint_tolerance");
  // if (current_wp_index_ == 0 &&
  //     dist(xhat_.position, waypoint_list_[0].w) > waypoint_tolerance) {
  //   roscopter_msgs::msg::Waypoint wp = waypoint_list_[0];
  //   output_cmd.position = wp.w;
  //   output_cmd.psi_cmd = wp.psi;

  //   RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(xhat_.position[0]) << " " << std::to_string(wp.w[0]));
  //   RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(xhat_.position[1]) << " " << std::to_string(wp.w[1]));
  //   RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(xhat_.position[2]) << " " << std::to_string(wp.w[2]));
  //   std::array<float, 3> array1 = wp.w;
  //   std::array<float, 3> array2 = xhat_.position;
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Norm: " << std::to_string(sqrt(pow(array1[0] - array2[0], 2) + pow(array1[1] - array2[1], 2) + pow(array1[2] - array2[2], 2))));
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Achieved first waypoint within " << std::to_string(waypoint_tolerance) << "m! distance: " << std::to_string(dist(xhat_.position, waypoint_list_[0].w) > waypoint_tolerance));

  //   return output_cmd;
  // }

  // Otherwise, compute the control according to Algorithm 15 in Beard, McLain textbook
  if (sigma_(0) >= 1.0) {
    // Update the path parameter to go to the next waypoint
    previous_wp_index_ = current_wp_index_;
    current_wp_index_ = (current_wp_index_ + 1) % waypoint_list_.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "Current wp: " << std::to_string(current_wp_index_) << " Previous wp: " << std::to_string(previous_wp_index_));

    double s1 = waypoint_list_[previous_wp_index_].speed;
    sigma_(0) = 0;
    sigma_(1) = s1 / dist(waypoint_list_[current_wp_index_].w, waypoint_list_[previous_wp_index_].w);
  }

  // Integrate the path parameter
  rk4_step();

  // Compute commanded trajectory
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];
  roscopter_msgs::msg::Waypoint prev_wp = waypoint_list_[previous_wp_index_];

  double s_i = curr_wp.speed;
  double s_i1 = prev_wp.speed;
  double norm = dist(curr_wp.w, prev_wp.w);
  double sigma_dot_1 = (pow(s_i, 2) - pow(s_i1, 2)) / (2 * pow(norm, 2));

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

void PathManager::rk4_step()
{
  double dt = params.get_double("path_update_dt");

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
  sigma_dot(1) = (pow(s_i, 2) - pow(s_i1, 2)) / (2 * pow(norm, 2));

  return sigma_dot;
}

} // namespace roscopter