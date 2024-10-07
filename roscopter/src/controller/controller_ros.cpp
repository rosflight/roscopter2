#include <controller/controller_ros.hpp>
#include <controller/controller_cascading_pid.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerROS::ControllerROS() : Node("controller"), params(this)
{
  // Set up Publisher and Subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 10, std::bind(&ControllerROS::state_callback, this, _1));
  cmd_sub_ = this->create_subscription<roscopter_msgs::msg::ControllerCommand>("high_level_command", 10, std::bind(&ControllerROS::cmd_callback, this, _1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>("status", 10, std::bind(&ControllerROS::status_callback, this, _1));
  command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 10);

  // Make sure the input command is initialized correctly to avoid sending commands before the controller receives control setpoints
  input_cmd_.cmd_valid = false;

  // Declare params with ROS and the ParamManager
  declare_params();
  params.set_parameters();

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ControllerROS::parameters_callback, this, _1));
}

rcl_interfaces::msg::SetParametersResult ControllerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of the controller.";

  // Use the ParamManager's set parameters callback
  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  // Call the update_gains method implemented in a derived class to save the gains in member variables there.
  update_gains();

  return result;
} 

void ControllerROS::declare_params()
{
  // Put any params needed in controller_base here
}

void ControllerROS::cmd_callback(const roscopter_msgs::msg::ControllerCommand &msg)
{
  input_cmd_ = msg;
}

void ControllerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Started receiving estimated state message.");

  // Save the estimated state for use in derived classes
  xhat_ = msg;

  // Calculate dt and return if dt is invalid
  double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  double dt = compute_dt(now);

  // Compute the control to send to the firmware by calling the state machine's manage_state method
  rosflight_msgs::msg::Command output_command = manage_state(input_cmd_, status_, dt);

  // Publish the control commands
  publish_command(output_command);
}

double ControllerROS::compute_dt(double now)
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

void ControllerROS::status_callback(const rosflight_msgs::msg::Status &msg)
{
  status_ = msg;
}

void ControllerROS::publish_command(rosflight_msgs::msg::Command &command)
{
  command.header.stamp = this->get_clock()->now();
  command_pub_->publish(command);
}

double ControllerROS::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}  // namespace controller


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter::ControllerCascadingPID>();
  RCLCPP_INFO_ONCE(node->get_logger(), "Using default (cascading PID) controller");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
