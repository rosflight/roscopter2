#include <controller/controller_cascading_pid.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerCascadingPID::ControllerCascadingPID() : ControllerStateMachine(), params_initialized_(false)
{
  // Declare and set parameters associated with Cascading PID controller
  declare_params();
  params.set_parameters();

  // Check to make sure equilibrium throttle is set properly
  if (!params.get_double("equilibrium_throttle")) {
    RCLCPP_ERROR(this->get_logger(), "Controller MAV equilibrium throttle not found!");
  }

  update_gains();
  reset_integrators();

  params_initialized_ = true;
  // Update gains after all the parameters have been set to initialize the PID controllers
  update_gains();
}

void ControllerCascadingPID::declare_params() {
  params.declare_double("roll_kp", 0.2);
  params.declare_double("roll_ki", 0.0);
  params.declare_double("roll_kd", 0.2);

  params.declare_double("pitch_kp", 0.2);
  params.declare_double("pitch_ki", 0.0);
  params.declare_double("pitch_kd", 0.2);

  params.declare_double("yaw_kp", 0.2);
  params.declare_double("yaw_ki", 0.002);
  params.declare_double("yaw_kd", 0.2);

  params.declare_double("roll_rate_kp", 0.2);
  params.declare_double("roll_rate_ki", 0.0);
  params.declare_double("roll_rate_kd", 0.2);

  params.declare_double("pitch_rate_kp", 0.2);
  params.declare_double("pitch_rate_ki", 0.0);
  params.declare_double("pitch_rate_kd", 0.2);

  params.declare_double("yaw_rate_kp", 0.2);
  params.declare_double("yaw_rate_ki", 0.0);
  params.declare_double("yaw_rate_kd", 0.2);

  params.declare_double("yaw_to_rate_kp", 0.2);
  params.declare_double("yaw_to_rate_ki", 0.0);
  params.declare_double("yaw_to_rate_kd", 0.2);

  params.declare_double("vel_n_P", 0.5);
  params.declare_double("vel_n_I", 0.0);
  params.declare_double("vel_n_D", 0.05);

  params.declare_double("vel_e_P", 0.5);
  params.declare_double("vel_e_I", 0.0);
  params.declare_double("vel_e_D", 0.05);

  params.declare_double("vel_d_P", 0.4);
  params.declare_double("vel_d_I", 0.25);
  params.declare_double("vel_d_D", 0.1);

  params.declare_double("n_P", 1.0);
  params.declare_double("n_I", 0.1);
  params.declare_double("n_D", 0.35);

  params.declare_double("e_P", 1.0);
  params.declare_double("e_I", 0.1);
  params.declare_double("e_D", 0.2);

  params.declare_double("d_P", 1.0);
  params.declare_double("d_I", 0.0);
  params.declare_double("d_D", 0.0);

  // System parameters
  params.declare_double("tau", 0.05);
  params.declare_double("mass", 2.0);
  params.declare_double("gravity", 9.81);

  // Saturation Limit parameters
  params.declare_double("max_roll_rate", 1.0);
  params.declare_double("max_pitch_rate", 1.0);
  params.declare_double("max_yaw_rate", 1.0);

  params.declare_double("max_roll_torque", 10.0);
  params.declare_double("max_pitch_torque", 10.0);
  params.declare_double("max_yaw_torque", 10.0);

  params.declare_double("max_n_vel", 10.0);
  params.declare_double("max_e_vel", 10.0);
  params.declare_double("max_d_vel", 10.0);

  // TODO: These could be replaced using an equation with the eq_throttle.
  // See the max_accel_xy calculation for example.
  params.declare_double("max_roll", 0.30);  
  params.declare_double("max_pitch", 0.30);

  params.declare_double("max_descend_accel", 1.0);
  params.declare_double("max_descend_rate", 3.0);

  params.declare_double("equilibrium_throttle", 0.45); 
  params.declare_double("max_throttle", 0.85);
  params.declare_double("min_throttle", 0.05);

  params.declare_double("min_altitude_for_attitude_ctrl", 0.15);
}

void ControllerCascadingPID::update_gains() {
  // Don't update gains if parameters are not initialized.
  if (!params_initialized_) { return; }

  RCLCPP_INFO_STREAM(this->get_logger(), "Updating gains!");

  double P, I, D, tau, max;
  tau = params.get_double("tau");

  // Roll PID loop
  P = params.get_double("roll_kp");
  I = params.get_double("roll_ki");
  D = params.get_double("roll_kd");
  max = params.get_double("max_roll_torque");
  PID_roll_.set_gains(P, I, D, tau, max, -max);

  // Pitch to torque PID loop
  P = params.get_double("pitch_kp");
  I = params.get_double("pitch_ki");
  D = params.get_double("pitch_kd");
  max = params.get_double("max_pitch_torque");
  PID_pitch_.set_gains(P, I, D, tau, max, -max);

  // Yaw to torque PID loop
  P = params.get_double("yaw_kp");
  I = params.get_double("yaw_ki");
  D = params.get_double("yaw_kd");
  max = params.get_double("max_yaw_torque");
  PID_yaw_.set_gains(P, I, D, tau, max, -max);

  // Roll rate to torque PID loop
  P = params.get_double("roll_rate_kp");
  I = params.get_double("roll_rate_ki");
  D = params.get_double("roll_rate_kd");
  max = params.get_double("max_roll_torque");
  PID_roll_rate_.set_gains(P, I, D, tau, max, -max);

  // Pitch rate to torque PID loop
  P = params.get_double("pitch_rate_kp");
  I = params.get_double("pitch_rate_ki");
  D = params.get_double("pitch_rate_kd");
  max = params.get_double("max_pitch_torque");
  PID_pitch_rate_.set_gains(P, I, D, tau, max, -max);

  // Yaw rate to torque PID loop
  P = params.get_double("yaw_rate_kp");
  I = params.get_double("yaw_rate_ki");
  D = params.get_double("yaw_rate_kd");
  max = params.get_double("max_yaw_torque");
  PID_yaw_rate_.set_gains(P, I, D, tau, max, -max);

  // PID loop from yaw to yaw rate
  P = params.get_double("yaw_to_rate_kp");
  I = params.get_double("yaw_to_rate_ki");
  D = params.get_double("yaw_to_rate_kd");
  max = params.get_double("max_yaw_rate");
  PID_yaw_to_rate_.set_gains(P, I, D, tau, max, -max);

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double max_accel_z = 1.0 / equilibrium_throttle;
  double max_accel_xy = calculate_max_xy_accel(max_accel_z, equilibrium_throttle);

  // North velocity to accel PID loop
  P = params.get_double("vel_n_P");
  I = params.get_double("vel_n_I");
  D = params.get_double("vel_n_D");
  max = max_accel_xy;
  PID_vel_n_.set_gains(P, I, D, tau, max, -max);

  // East velocity to accel PID loop
  P = params.get_double("vel_e_P");
  I = params.get_double("vel_e_I");
  D = params.get_double("vel_e_D");
  max = max_accel_xy;
  PID_vel_e_.set_gains(P, I, D, tau, max, -max);

  // Down velocity to accel PID loop
  P = params.get_double("vel_d_P");
  I = params.get_double("vel_d_I");
  D = params.get_double("vel_d_D");
  // set max z accelerations so that we can't fall faster than 1 gravity
  max = max_accel_z;
  double min = params.get_double("max_descend_accel");
  PID_vel_d_.set_gains(P, I, D, tau, min, -max);

  // North position to velocity PID loop
  P = params.get_double("n_P");
  I = params.get_double("n_I");
  D = params.get_double("n_D");
  max = params.get_double("max_n_vel");
  PID_n_.set_gains(P, I, D, tau, max, -max);

  // East position to velocity PID loop
  P = params.get_double("e_P");
  I = params.get_double("e_I");
  D = params.get_double("e_D");
  max = params.get_double("max_e_vel");
  PID_e_.set_gains(P, I, D, tau, max, -max);

  // Down position to velocity PID loop
  P = params.get_double("d_P");
  I = params.get_double("d_I");
  D = params.get_double("d_D");
  max = params.get_double("max_d_vel");
  min = params.get_double("max_descend_rate");
  PID_d_.set_gains(P, I, D, tau, min, -max);
}

double ControllerCascadingPID::calculate_max_xy_accel(double max_accel_z, double equilibrium_throttle) {
  // Compute the maximum acceleration in the xy plane based on the maximum z accel
  double max_accel_xy = sin(acos(equilibrium_throttle)) 
    * max_accel_z; // This assumes that the minimum vehicle-1 frame z acceleration is 1g

  // Also compute the max acceleration that doesn't exceed the max roll and pitch values
  double max_accel_due_to_roll = sin(params.get_double("max_roll") * TO_RADIANS) * max_accel_z;
  double max_accel_due_to_pitch = sin(params.get_double("max_pitch") * TO_RADIANS) * max_accel_z;

  // Take the minimum of the computed maximum accelerations
  max_accel_xy = std::min(max_accel_xy, max_accel_due_to_roll);
  max_accel_xy = std::min(max_accel_xy, max_accel_due_to_pitch);

  RCLCPP_INFO_STREAM(this->get_logger(), "Max accel in xy: " << max_accel_xy << " Max accel in z: " << max_accel_z);

  return max_accel_xy;
}

rosflight_msgs::msg::Command ControllerCascadingPID::compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt)
{
  // Note that dt is "safe", i.e., >0.0000001 as checked in the state machine
  dt_ = dt;

  uint8_t mode = input_cmd.mode;

  switch (mode) {
    case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW:
      npos_epos_dpos_yaw(input_cmd);
      break;
    
    case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DPOS_YAWRATE:
      nvel_evel_dpos_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE:
      nacc_eacc_dacc_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE:
      nvel_evel_dvel_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW:
      npos_epos_dvel_yaw(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THROTTLE:
      roll_pitch_yaw_throttle(input_cmd);
      break;
    
    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THRUST_TO_MOTOR:
      roll_pitch_yaw_thrust_to_motor(input_cmd);
      break;
    
    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THRUST_TO_MOTOR:
      rollrate_pitchrate_yawrate_thrust_to_motor(input_cmd);
      break;
    
    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH:
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      pass_to_firmware_controller(input_cmd);
      break;

    default:
      // TODO: Is this desired behavior?
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown control mode! Using previous command (if any)...");
      break;
  }

  return output_cmd_;
}

void ControllerCascadingPID::roll_pitch_yaw_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double phi = input_cmd.cmd1;
  double theta = input_cmd.cmd2;
  double psi = input_cmd.cmd3;

  // Update input command values (except thrust)
  input_cmd.cmd1 = PID_roll_.compute_pid(phi, xhat_.phi, dt_, xhat_.p);
  input_cmd.cmd2 = PID_pitch_.compute_pid(theta, xhat_.theta, dt_, xhat_.q);
  input_cmd.cmd3 = PID_yaw_.compute_pid(psi, xhat_.psi, dt_, xhat_.r);
  
  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::rollrate_pitchrate_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double p = input_cmd.cmd1;
  double q = input_cmd.cmd2;
  double r = input_cmd.cmd3;

  // Update input command values
  input_cmd.cmd1 = PID_roll_rate_.compute_pid(p , xhat_.p, dt_);
  input_cmd.cmd2 = PID_pitch_rate_.compute_pid(q, xhat_.q, dt_);
  input_cmd.cmd3 = PID_yaw_rate_.compute_pid(r, xhat_.r, dt_);
  
  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double pn = input_cmd.cmd1;
  double pe = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double psi = input_cmd.cmd4;

  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt_);
  double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt_);
  double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt_);

  // Calculate desired yaw rate
  // TOOD: Wrap to within 180 deg
  // First, determine the shortest direction to the commanded psi
  if(fabs(psi + 2*M_PI - xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi += 2*M_PI;
  }
  else if (fabs(psi - 2*M_PI -xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi -= 2*M_PI;
  }

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  // Convert desired inertial velocities to vehicle-1 frame
  input_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);   // n_dot in vehicle-1 frame
  input_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);  // e_dot in vehicle-1 frame
  input_cmd.cmd3 = pddot_c;                                           // d_dot in vehicle-1 frame
  input_cmd.cmd4 = PID_yaw_to_rate_.compute_pid(psi, xhat_.psi, dt_);         // r in vehicle-1 frame

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // In vehicle-1 frame
  double vel_n = input_cmd.cmd1;
  double vel_e = input_cmd.cmd2;
  double vel_d = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Rotate inertial frame velocities to vehicle 1 frame velocities
  double sin_psi = sin(xhat_.psi);
  double cos_psi = cos(xhat_.psi);
  
  double pxdot = cos_psi * xhat_.v_n + sin_psi * xhat_.v_e;
  double pydot = -sin_psi * xhat_.v_n + cos_psi * xhat_.v_e;
  double pddot = xhat_.v_d;

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

  // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
  input_cmd.cmd1 = PID_vel_n_.compute_pid(vel_n, pxdot, dt_);  // ax
  input_cmd.cmd2 = PID_vel_e_.compute_pid(vel_e, pydot, dt_);  // ay
  input_cmd.cmd3 = PID_vel_d_.compute_pid(vel_d, pddot, dt_);  // az
  input_cmd.cmd4 = r;                                          // r

  nacc_eacc_dacc_yawrate(input_cmd);
}

void ControllerCascadingPID::nacc_eacc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double max_roll = params.get_double("max_roll");
  double max_pitch = params.get_double("max_pitch");

  // In vehicle-1 frame, units of g's
  double ax = input_cmd.cmd1;
  double ay = input_cmd.cmd2;
  double az = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  double phi = 0.0;
  double theta = 0.0;

  // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
  double total_acc_c = sqrt((1.0 - az) * (1.0 - az) +
                            ax * ax + ay * ay);  // (in g's)
  // Avoid dividing by zero
  if (total_acc_c > 0.001)
  {
    phi = asin(ay / total_acc_c);
    theta = -1.0*asin(ax / total_acc_c);
  }
  else
  {
    phi = 0;
    theta = 0;
  }

  // Compute desired thrust based on current pose (with mass divided out)
  double cosp = cos(xhat_.phi);
  double cost = cos(xhat_.theta);
  double desired_thrust = (1.0 - az) / cosp / cost;

  // Save the calculated values to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  input_cmd.cmd1 = saturate(phi, max_roll, -max_roll);         // phi
  input_cmd.cmd2 = saturate(theta, max_pitch, -max_pitch);     // theta
  input_cmd.cmd3 = r;                                          // r
  input_cmd.cmd4 = desired_thrust * equilibrium_throttle;      // throttle

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::npos_epos_dvel_yaw(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double pn = input_cmd.cmd1;
  double pe = input_cmd.cmd2;
  double vd = input_cmd.cmd3;
  double psi = input_cmd.cmd4;

  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt_);
  double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt_);

  // Calculate desired yaw rate
  // First, determine the shortest direction to the commanded psi
  if(fabs(psi + 2*M_PI - xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi += 2*M_PI;
  }
  else if (fabs(psi - 2*M_PI -xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi -= 2*M_PI;
  }

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  // In vehicle-1 frame
  input_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);   // vel_n
  input_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);  // vel_e
  input_cmd.cmd3 = vd;                                                // vel_d
  input_cmd.cmd4 = PID_yaw_to_rate_.compute_pid(psi, xhat_.psi, dt_); // r

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double vel_n = input_cmd.cmd1;
  double vel_e = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
  // Rotate inertial frame velocities to vehicle 1 frame velocities
  double sin_psi = sin(xhat_.psi);
  double cos_psi = cos(xhat_.psi);
  
  double pxdot = cos_psi * xhat_.v_n + sin_psi * xhat_.v_e;
  double pydot = -sin_psi * xhat_.v_n + cos_psi * xhat_.v_e;
  double pddot = xhat_.v_d;

  // Nested Loop for Altitude - from desired position to desired velocity
  double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt_, pddot);

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

  input_cmd.cmd1 = PID_vel_n_.compute_pid(vel_n, pxdot, dt_);   // ax
  input_cmd.cmd2 = PID_vel_e_.compute_pid(vel_e, pydot, dt_);   // ay
  input_cmd.cmd3 = PID_vel_d_.compute_pid(pddot_c, pddot, dt_); // az
  input_cmd.cmd4 = r;                                           // r

  nacc_eacc_dacc_yawrate(input_cmd);
}

void ControllerCascadingPID::roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Replace the yaw control command with the calculated yawrate command
  double yaw_cmd = input_cmd.cmd3;
  input_cmd.cmd3 = PID_yaw_to_rate_.compute_pid(yaw_cmd, xhat_.psi, dt_, xhat_.r);

  // Assign the correct control mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Saturate and return the output command object
  double min_altitude_for_attitude_ctrl = params.get_double("min_altitude_for_attitude_ctrl");
  double max_x, max_y, max_z, max_f, min_f;
  max_f = params.get_double("max_throttle");
  min_f = params.get_double("min_throttle");

  // Set the mode and saturation limits
  if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
    max_x = params.get_double("max_roll_torque");
    max_y = params.get_double("max_pitch_torque");
    max_z = params.get_double("max_yaw_torque");
    
    // Compute the max and min force based on the equilibrium throttle
    double equilibrium_throttle = params.get_double("equilibrium_throttle");
    double mass = params.get_double("mass");
    double gravity = params.get_double("gravity");
    max_f = mass * gravity / equilibrium_throttle * max_f;
    min_f = mass * gravity / equilibrium_throttle * min_f;
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    max_x = params.get_double("max_roll_rate");
    max_y = params.get_double("max_pitch_rate");
    max_z = params.get_double("max_yaw_rate");
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    max_x = params.get_double("max_roll");
    max_y = params.get_double("max_pitch");
    max_z = params.get_double("max_yaw_rate");
  }
  else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unknown command input type! Using previous command (if any)...");
    max_x = 0.0;
    max_y = 0.0;
    max_z = 0.0;
    return;
  }

  output_cmd_.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;

  // Saturate the commanded output
  output_cmd_.qx = saturate(input_cmd.cmd1, max_x, -max_x);
  output_cmd_.qy = saturate(input_cmd.cmd2, max_y, -max_y);
  output_cmd_.qz = saturate(input_cmd.cmd3, max_z, -max_z);
  output_cmd_.fz = saturate(input_cmd.cmd4, max_f, min_f);
  output_cmd_.fx = 0.0;
  output_cmd_.fy = 0.0;

  // Check to see if we are above the minimum attitude altitude
  if (abs(xhat_.position[2]) < min_altitude_for_attitude_ctrl)
  {
    output_cmd_.qx = 0.;
    output_cmd_.qy = 0.;
    output_cmd_.qz = 0.;
  }
}

void ControllerCascadingPID::reset_integrators()
{
  PID_vel_n_.clear_integrator();
  PID_vel_e_.clear_integrator();
  PID_vel_d_.clear_integrator();

  PID_n_.clear_integrator();
  PID_e_.clear_integrator();
  PID_d_.clear_integrator();

  PID_roll_.clear_integrator();
  PID_pitch_.clear_integrator();
  PID_yaw_.clear_integrator();

  PID_roll_rate_.clear_integrator();
  PID_pitch_rate_.clear_integrator();
  PID_yaw_rate_.clear_integrator();
}

}  // namespace controller

