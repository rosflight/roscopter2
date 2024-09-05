#include "navigation/path_manager_ros.hpp"
#include "navigation/path_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace roscopter
{

PathManagerROS::PathManagerROS() : Node("path_manager"), params(this)
{
  // Instantiate publishers and subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&PathManagerROS::state_callback, this, _1));
  cmd_pub_ = this->create_publisher<roscopter_msgs::msg::TrajectoryCommand>("trajectory_command", 1);
  
  // Instantiate service servers
  single_waypoint_srv_ = this->create_service<roscopter_msgs::srv::AddWaypoint>("path_manager/add_waypoint", std::bind(&PathManagerROS::single_waypoint_callback, this, _1, _2));
  waypoint_list_srv_ = this->create_service<roscopter_msgs::srv::AddWaypointList>("path_manager/add_waypoint_list", std::bind(&PathManagerROS::set_waypoint_list, this, _1, _2));

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PathManagerROS::parameters_callback, this, _1));
  
  declare_params();
  params.set_parameters();
}

void PathManagerROS::declare_params()
{
  // Declare params here needed in this node
}

rcl_interfaces::msg::SetParametersResult PathManagerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of the trajectory follower.";

  // Use the ParamManager's set parameters callback
  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  return result;
}

void PathManagerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  xhat_ = msg;

  double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  double dt = compute_dt(now);

  roscopter_msgs::msg::TrajectoryCommand output_cmd = manage_path(dt);

  publish_command(output_cmd);
}

double PathManagerROS::compute_dt(double now)
{
  static double prev_time = 0;
  if(prev_time == 0) {
    prev_time = now;
    // Don't compute control since we don't have a dt calculation
    return 0;
  }

  // Calculate time
  double dt = now - prev_time;
  prev_time = now;

  return dt;
}

bool PathManagerROS::single_waypoint_callback(const roscopter_msgs::srv::AddWaypoint::Request::SharedPtr &req,
                                              const roscopter_msgs::srv::AddWaypoint::Response::SharedPtr &res)
{
  waypoint_list_.push_back(req->wp);

  res->success = true;
return true;
}

bool PathManagerROS::set_waypoint_list(const roscopter_msgs::srv::AddWaypointList::Request::SharedPtr &req,
                                       const roscopter_msgs::srv::AddWaypointList::Response::SharedPtr &res)
{
  if (req->clear_previous) {
    waypoint_list_.clear();
  }

  // Add the waypoints to the list of waypoints
  for (auto wp : req->wp_list) {
    waypoint_list_.push_back(wp);
  }

  if (waypoint_list_.size() == 1) {
    // If there is only one waypoint in the list, add the current state as a waypoint
    roscopter_msgs::msg::Waypoint self_wp;
    self_wp.w = xhat_.position;
    self_wp.speed = xhat_.vg;
    self_wp.psi = xhat_.psi;

    waypoint_list_.insert(waypoint_list_.begin(), self_wp);
  }

  res->success = true;

  return true;
}

void PathManagerROS::publish_command(roscopter_msgs::msg::TrajectoryCommand &command)
{
  command.header.stamp = this->get_clock()->now();
  cmd_pub_->publish(command);
}

}   // namespace roscopter


int main(int argc, char* argv[])
{
    // Initialize the ROS2 client library
    rclcpp::init(argc, argv);

    // Create and spin node
    auto node = std::make_shared<roscopter::PathManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}