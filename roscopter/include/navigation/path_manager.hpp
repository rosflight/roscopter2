#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include "navigation/path_manager_ros.hpp"
#include "roscopter_msgs/msg/waypoint.hpp"
#include "roscopter_msgs/msg/trajectory_command.hpp"

#include <Eigen/Core>

#include <vector>
#include <cmath>

namespace roscopter {

class PathManager : public PathManagerROS
{
public:
  PathManager();

private:
  // Methods
  roscopter_msgs::msg::TrajectoryCommand manage_path() override;
  void declare_params();
  roscopter_msgs::msg::TrajectoryCommand manage_goto_wp(roscopter_msgs::msg::Waypoint &curr_wp);
  roscopter_msgs::msg::TrajectoryCommand manage_hold_wp(roscopter_msgs::msg::Waypoint &curr_wp);
  void hold_timer_callback();
  void increment_wp_index();
  void clear_waypoints_internally() override;

  // Member variables
  rclcpp::TimerBase::SharedPtr hold_timer_;
  bool timer_started_;
  int current_wp_index_;
};

} // namespace roscopter

#endif