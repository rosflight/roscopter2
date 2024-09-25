#include "navigation/path_manager_ros.hpp"
#include "navigation/path_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace roscopter
{

PathManagerROS::PathManagerROS() : Node("path_manager"), params(this), params_initialized_(false)
{
  rclcpp::QoS qos_transient_local_10_(10);
  qos_transient_local_10_.transient_local();

  // Instantiate publishers and subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&PathManagerROS::state_callback, this, _1));
  wp_sub_ = this->create_subscription<roscopter_msgs::msg::Waypoint>("waypoints", qos_transient_local_10_, std::bind(&PathManagerROS::single_waypoint_callback, this, _1));
  cmd_pub_ = this->create_publisher<roscopter_msgs::msg::TrajectoryCommand>("trajectory_command", 1);
  
  // Instantiate service servers
  clear_waypoints_srv_ = this->create_service<std_srvs::srv::Trigger>("path_manager/clear_waypoints", std::bind(&PathManagerROS::clear_waypoints, this, _1, _2));
  print_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "path_manager/print_waypoints", std::bind(&PathManagerROS::print_path, this, _1, _2));

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PathManagerROS::parameters_callback, this, _1));
  
  declare_params();
  params.set_parameters();

  params_initialized_ = true;
  set_timer();
}

void PathManagerROS::declare_params()
{
  // Declare params here needed in this node
  params.declare_double("path_update_frequency", 50.0);
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


  if (params_initialized_ && success) {
    std::chrono::microseconds curr_period = std::chrono::microseconds(
      static_cast<long long>(1.0 / params.get_double("path_update_frequency") * 1'000'000));
    if (timer_period_ != curr_period) {
      timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void PathManagerROS::set_timer()
{
  timer_period_ = std::chrono::microseconds(
    static_cast<long long>(1.0 / params.get_double("path_update_frequency") * 1'000'000));

  timer_ = this->create_wall_timer(timer_period_,
                                   std::bind(&PathManagerROS::run, this));
}

void PathManagerROS::run()
{
  roscopter_msgs::msg::TrajectoryCommand output_cmd = manage_path();

  publish_command(output_cmd);
}

void PathManagerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  xhat_ = msg;
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

void PathManagerROS::single_waypoint_callback(const roscopter_msgs::msg::Waypoint &msg)
{
  waypoint_list_.push_back(msg);
}

// bool PathManagerROS::set_waypoint_list(const roscopter_msgs::srv::AddWaypointList::Request::SharedPtr &req,
//                                        const roscopter_msgs::srv::AddWaypointList::Response::SharedPtr &res)
// {
//   if (req->clear_previous) {
//     waypoint_list_.clear();
//   }

//   // Add the waypoints to the list of waypoints
//   for (auto wp : req->wp_list) {
//     waypoint_list_.push_back(wp);
//   }

//   if (waypoint_list_.size() == 1) {
//     // If there is only one waypoint in the list, add the current state as a waypoint
//     roscopter_msgs::msg::Waypoint self_wp;
//     self_wp.w = xhat_.position;
//     self_wp.speed = xhat_.vg;
//     self_wp.psi = xhat_.psi;

//     waypoint_list_.insert(waypoint_list_.begin(), self_wp);
//   }

//   res->success = true;

//   return true;
// }

bool PathManagerROS::clear_waypoints(const std_srvs::srv::Trigger::Request::SharedPtr &req,
                                     const std_srvs::srv::Trigger::Response::SharedPtr &res)
{
  clear_waypoints_internally();
  
  res->success = true;
  res->message = "Waypoints cleared!";
  return true;
}

void PathManagerROS::publish_command(roscopter_msgs::msg::TrajectoryCommand &command)
{
  command.header.stamp = this->get_clock()->now();
  cmd_pub_->publish(command);
}

bool PathManagerROS::print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  std::stringstream output;

  output << "Printing waypoints...";

  for (int i = 0; i < (int) waypoint_list_.size(); ++i) {
    roscopter_msgs::msg::Waypoint wp = waypoint_list_[i];
    output << std::endl << "----- WAYPOINT " << i << " -----" << std::endl;
    output << "Type (HOLD/GOTO): " << (int)wp.type << std::endl;

    if (wp.use_lla) {
      output << "Position (LLA): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]"
             << std::endl;
    } else {
      output << "Position (NED, meters): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]"
             << std::endl;
    }
    output << "Speed: " << wp.speed << std::endl;
    output << "Psi: " << wp.psi << std::endl;
    output << "Hold Seconds: " << wp.hold_seconds << std::endl;
    output << "Hold Indefinitely: " << wp.hold_indefinitely;
  }

  // Print to info log stream
  RCLCPP_INFO_STREAM(this->get_logger(), output.str());

  res->success = true;

  return true;
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