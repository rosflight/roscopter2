#include "navigation/path_planner.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace roscopter
{

PathPlanner::PathPlanner()
    : Node("path_planner")
    , params_(this)
{
  // Set up the callback groups for the clear wp service and the service client so they can execute properly
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clear_wp_client_ = this->create_client<std_srvs::srv::Trigger>("/path_manager/clear_waypoints", rmw_qos_profile_services_default, client_cb_group_);

  // Make this publisher transient_local so that it publishes the last 10 waypoints to late subscribers
  rclcpp::QoS qos_transient_local_10_(10);
  qos_transient_local_10_.transient_local();
  waypoint_publisher_ =
    this->create_publisher<roscopter_msgs::msg::Waypoint>("waypoints", qos_transient_local_10_);

  next_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "publish_next_waypoint", std::bind(&PathPlanner::publish_next_waypoint, this, _1, _2));

  add_waypoint_service_ = this->create_service<roscopter_msgs::srv::AddWaypoint>(
    "add_waypoint", std::bind(&PathPlanner::update_path, this, _1, _2));

  clear_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_waypoints", std::bind(&PathPlanner::clear_path_callback, this, _1, _2));

  print_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "print_waypoints", std::bind(&PathPlanner::print_path, this, _1, _2));

  load_mission_service_ = this->create_service<rosflight_msgs::srv::ParamFile>(
    "load_mission_from_file", std::bind(&PathPlanner::load_mission, this, _1, _2));

  state_subscription_ = this->create_subscription<roscopter_msgs::msg::State>(
    "estimated_state", 10, std::bind(&PathPlanner::state_callback, this, _1));

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PathPlanner::parametersCallback, this, std::placeholders::_1));

  // Declare parameters with ROS2 and save them to the param_manager object
  declare_parameters();
  params_.set_parameters();

  num_waypoints_published_ = 0;

  // Initialize by publishing a clear path command.
  // This makes sure rviz or other vizualization tools don't show stale waypoints if ROScopter is restarted.
  // clear_path();

  // Publishes the initial waypoints
  publish_initial_waypoints();
}

PathPlanner::~PathPlanner() {}

void PathPlanner::publish_initial_waypoints()
{
  int num_waypoints_to_publish_at_start =
    this->get_parameter("num_waypoints_to_publish_at_start").as_int();

  RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                          "Path Planner will publish the first {"
                            << num_waypoints_to_publish_at_start << "} available waypoints!");

  // Publish the first waypoints as defined by the num_waypoints_to_publish_at_start parameter
  while (num_waypoints_published_ < num_waypoints_to_publish_at_start
         && num_waypoints_published_ < (int) wps_.size()) {
    waypoint_publish();
  }
}

void PathPlanner::state_callback(const roscopter_msgs::msg::State & msg)
{
  // Make sure initial LLA is not zero before updating to avoid initialization errors
  // TODO: What if we want to initialize it at (0,0,0)?
  if (fabs(msg.initial_lat) > 0.0 || fabs(msg.initial_lon) > 0.0 || fabs(msg.initial_alt) > 0.0) {
    initial_lat_ = msg.initial_lat;
    initial_lon_ = msg.initial_lon;
    initial_alt_ = msg.initial_alt;
  }
}

bool PathPlanner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                        const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  // Publish the next waypoint, if available
  if (num_waypoints_published_ < (int) wps_.size()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published_ + 1);

    waypoint_publish();

    res->success = true;
    return true;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints left to publish! Add more waypoints");
    res->success = false;
    return false;
  }
}

void PathPlanner::waypoint_publish()
{
  // Publish the next waypoint off the list
  roscopter_msgs::msg::Waypoint new_waypoint = wps_[num_waypoints_published_];

  waypoint_publisher_->publish(new_waypoint);

  num_waypoints_published_++;
}

bool PathPlanner::update_path(const roscopter_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                              const roscopter_msgs::srv::AddWaypoint::Response::SharedPtr & res)
{

  roscopter_msgs::msg::Waypoint new_waypoint;

  rclcpp::Time now = this->get_clock()->now();

  new_waypoint.header.stamp = now;

  // Create a string flag for the debug message (defaults to "LLA")
  std::string lla_or_ned = "LLA";

  // Convert to NED if given in LLA
  if (req->wp.use_lla) {
    std::array<float, 3> ned = lla2ned(req->wp.w);
    new_waypoint.w = ned;
  } else {
    new_waypoint.w = req->wp.w;
    lla_or_ned = "NED";
  }

  // Fill in the Waypoint object with the information from the service request object
  new_waypoint.type = req->wp.type;
  new_waypoint.speed = req->wp.speed;
  new_waypoint.psi = req->wp.psi;
  new_waypoint.hold_seconds = req->wp.hold_seconds;
  new_waypoint.hold_indefinitely = req->wp.hold_indefinitely;
  new_waypoint.use_lla = req->wp.use_lla;

  if (req->publish_now) {
    // Insert the waypoint in the correct location in the list and publish it
    wps_.insert(wps_.begin() + num_waypoints_published_, new_waypoint);
    waypoint_publish();
    res->message = "Adding " + lla_or_ned + " waypoint was successful! Waypoint published.";
  } else {
    wps_.push_back(new_waypoint);
    res->message = "Adding " + lla_or_ned + " waypoint was successful!";
  }

  publish_initial_waypoints();

  res->success = true;
  return true;
}

bool PathPlanner::clear_path_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                      const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  res->success = clear_path();

  return true;
}

bool PathPlanner::clear_path()
{
  // Send a clear path service request to the path_manager
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!clear_wp_client_->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service not available");
    return false;
  }

  auto result_future = clear_wp_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(10s);    // Guarantees graceful finish

  if (status == std::future_status::ready) {
    if (result_future.get()->success == false) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Unable to clear wayoints from the path_manager. Not clearing waypoints from the path_planner.");
        return false;
    }

  } else { 
    RCLCPP_INFO_STREAM(this->get_logger(), "Failed to call /path_manager/clear_waypoints service. Is path_manager running?");
    return false;
  }

  // Otherwise, clear the internal waypoint list and reset the number of published waypoints
  RCLCPP_INFO_STREAM(this->get_logger(), "Testing!");
  wps_.clear();
  num_waypoints_published_ = 0;
  return true;
}

bool PathPlanner::print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  std::stringstream output;

  output << "Printing waypoints...";

  for (int i = 0; i < (int) wps_.size(); ++i) {
    roscopter_msgs::msg::Waypoint wp = wps_[i];
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

bool PathPlanner::load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                               const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res)
{
  clear_path();
  res->success = load_mission_from_file(req->filename);
  publish_initial_waypoints();
  return true;
}

bool PathPlanner::load_mission_from_file(const std::string & filename)
{
  try {
    YAML::Node root = YAML::LoadFile(filename);
    assert(root.IsSequence());
    // RCLCPP_INFO_STREAM(this->get_logger(), root);

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
      YAML::Node wp = it->second;
      RCLCPP_INFO_STREAM(this->get_logger(), wp);

      roscopter_msgs::msg::Waypoint new_wp;
      new_wp.type = wp["type"].as<int>();
      new_wp.w = wp["w"].as<std::array<float, 3>>();

      // If LLA, convert to NED
      if (wp["use_lla"].as<bool>()) {
        std::array<float, 3> ned = lla2ned(new_wp.w);
        new_wp.w = ned;
      }

      new_wp.speed = wp["speed"].as<double>();
      new_wp.psi = wp["psi"].as<double>();
      new_wp.hold_seconds = wp["hold_seconds"].as<double>();
      new_wp.hold_indefinitely = wp["hold_indefinitely"].as<bool>();

      wps_.push_back(new_wp);
    }

    return true;
  } catch (...) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error while parsing mission YAML file! Check inputs");
    return false;
  }
}

std::array<float, 3> PathPlanner::lla2ned(std::array<float, 3> lla)
{
  float lat1 = lla[0];
  float lon1 = lla[1];
  float alt1 = lla[2];

  float lat0 = initial_lat_;
  float lon0 = initial_lon_;
  float alt0 = initial_alt_;

  float n = EARTH_RADIUS * (lat1 - lat0) * M_PI / 180.0;
  float e = EARTH_RADIUS * cos(lat0 * M_PI / 180.0) * (lon1 - lon0) * M_PI / 180.0;
  float d = -(alt1 - alt0);

  // Usually will not be flying exactly at these locations.
  // If the GPS reports (0,0,0), it most likely means there is an error with the GPS
  if (fabs(initial_lat_) == 0.0 || fabs(initial_lon_) == 0.0 || fabs(initial_alt_) == 0.0) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "NED position set to ["
                         << n << "," << e << "," << d
                         << "]! Waypoints may be incorrect. Check GPS health");
  }

  return std::array<float, 3>{n, e, d};
}

rcl_interfaces::msg::SetParametersResult
PathPlanner::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the controller node.";

  // Set parameters in the param_manager object
  bool success = params_.set_parameters_callback(parameters);
  if (success) {
    result.successful = true;
    result.reason = "success";
  }

  return result;
}

void PathPlanner::declare_parameters()
{
  params_.declare_int("num_waypoints_to_publish_at_start", 3);
}

} // namespace roscopter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter::PathPlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
