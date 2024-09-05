#ifndef PATH_MANAGER_ROS_H
#define PATH_MANAGER_ROS_H

#include <rclcpp/rclcpp.hpp>

#include "roscopter_msgs/msg/waypoint.hpp"
#include "roscopter_msgs/msg/trajectory_command.hpp"
#include "roscopter_msgs/msg/state.hpp"
#include "roscopter_msgs/srv/add_waypoint_list.hpp"
#include "roscopter_msgs/srv/add_waypoint.hpp"

#include "param_manager/param_manager.hpp"

#include <stdint.h>
#include <vector>

using std::placeholders::_1;

namespace roscopter
{

class PathManagerROS : public rclcpp::Node
{

public:

  PathManagerROS();

protected:
  ParamManager params;  
  roscopter_msgs::msg::State xhat_;     /** Current estimated state of MAV */
  std::vector<roscopter_msgs::msg::Waypoint> waypoint_list_;

private:
  // Publishers and Subscribers
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<roscopter_msgs::msg::TrajectoryCommand>::SharedPtr cmd_pub_;

  // Service servers
  rclcpp::Service<roscopter_msgs::srv::AddWaypoint>::SharedPtr single_waypoint_srv_;
  rclcpp::Service<roscopter_msgs::srv::AddWaypointList>::SharedPtr waypoint_list_srv_;

  // Functions
  double compute_dt(double now);
  void state_callback(const roscopter_msgs::msg::State &msg);
  void publish_command(roscopter_msgs::msg::TrajectoryCommand &command);
  bool single_waypoint_callback(const roscopter_msgs::srv::AddWaypoint::Request::SharedPtr &req,
                                const roscopter_msgs::srv::AddWaypoint::Response::SharedPtr &res);
  bool set_waypoint_list(const roscopter_msgs::srv::AddWaypointList::Request::SharedPtr &req,
                         const roscopter_msgs::srv::AddWaypointList::Response::SharedPtr &res);

  // Virtual functions
  virtual roscopter_msgs::msg::TrajectoryCommand manage_path(double dt) = 0;

  /**
   * @brief Declares parameters with ROS2 and loads from a parameter file, if given
   */
  void declare_params();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * @brief Callback function to handle parameter changes
   * 
   * @param parameters: Vector of rclcpp::Parameter objects that were changed
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

};
}

#endif
