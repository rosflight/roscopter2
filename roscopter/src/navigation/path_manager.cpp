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
{
  declare_params();
}

void PathManager::declare_params()
{
  params.declare_double("default_altitude", 10.0);
  params.declare_bool("hover_last", 0);
  params.declare_double("waypoint_tolerance", 10.0);
  params.declare_double("path_update_dt", 0.005);
  params.declare_bool("hold_last", true);
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_path()
{
  // Check the number of waypoints - do we have enough to do path management?
  if (waypoint_list_.size() < 1) {
    roscopter_msgs::msg::TrajectoryCommand output_cmd;

    double default_altitude = params.get_double("default_altitude");
    output_cmd.position[2] = -default_altitude;

    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints received! Orbiting the origin at " << std::to_string(default_altitude) << "!");
    return output_cmd;
  }
  else if (waypoint_list_.size() == 1) {
    // If only one waypoint is added, do a HOLD type trajectory
    roscopter_msgs::msg::Waypoint wp = waypoint_list_[0];
  
    return manage_goto_wp(wp);
  }
  
  // Otherwise, manage the correct type of waypoint
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];

  if (curr_wp.type == roscopter_msgs::msg::Waypoint::TYPE_GOTO) {
    return manage_goto_wp(curr_wp);
  }
  else {    // Type HOLD
    return manage_hold_wp(curr_wp);
  }
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_goto_wp(roscopter_msgs::msg::Waypoint &curr_wp) {
  roscopter_msgs::msg::TrajectoryCommand output_cmd;
  output_cmd.position = curr_wp.w;
  output_cmd.psi_cmd = curr_wp.psi;

  double waypoint_tolerance = params.get_double("waypoint_tolerance");
  if (dist(xhat_.position, curr_wp.w) <= waypoint_tolerance) {
    // If we are close enough to the target wayoint, increment the index in the waypoint list
    increment_wp_index();
  }

  return output_cmd;
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_hold_wp(roscopter_msgs::msg::Waypoint &curr_wp) {
  roscopter_msgs::msg::TrajectoryCommand output_cmd;

  double waypoint_tolerance = params.get_double("waypoint_tolerance");
  if (dist(xhat_.position, curr_wp.w) <= waypoint_tolerance && !curr_wp.hold_indefinitely) {
    // Start the hold timer, if applicable
    std::chrono::microseconds timer_period_ = std::chrono::microseconds(static_cast<long long>(curr_wp.hold_seconds * 1'000'000));
    if (!timer_started_) {
      hold_timer_ = this->create_wall_timer(timer_period_, std::bind(&PathManager::hold_timer_callback, this));
      timer_started_ = true;
    }
  }

  // Fill in the output command. Note that this is the same regardless of the conditions above.
  output_cmd.position = curr_wp.w;
  output_cmd.psi_cmd = curr_wp.psi;

  return output_cmd;
}

void PathManager::hold_timer_callback() {
  increment_wp_index();

  hold_timer_->cancel();
  timer_started_ = false;
}

void PathManager::increment_wp_index() {
  if ((current_wp_index_ + 1) % waypoint_list_.size() == 0) {
    if (!params.get_bool("hold_last")) {
      current_wp_index_ = 0;
    }
  }
  else {
    current_wp_index_++;
  }
}

void PathManager::clear_waypoints_internally() {
  waypoint_list_.clear();
  current_wp_index_ = 0;
}

} // namespace roscopter
