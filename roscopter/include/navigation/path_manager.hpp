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
  void rk4_step();
  Eigen::Vector2f F(Eigen::Vector2f sig);

  // Member variables
  Eigen::Vector2f sigma_;
  
  // Index trackers and flags
  int current_wp_index_;
  int previous_wp_index_;

  void declare_params();
};

} // namespace roscopter

#endif