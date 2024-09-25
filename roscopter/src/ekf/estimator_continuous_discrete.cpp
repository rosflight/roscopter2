#include <cmath>
#include <functional>
#include <rclcpp/logging.hpp>
#include <tuple>

#include "ekf/estimator_continuous_discrete.hpp"
#include "ekf/estimator_ros.hpp"
#include "ekf/geomag.h"

// TODO:Get rid of all MatrixX, only use deterministic sizes.

namespace roscopter
{

// ======== UTILITY FUNCTIONS ========
float radians(float degrees) { return M_PI * degrees / 180.0; }

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

// ======== CONSTRUCTOR ========
EstimatorContinuousDiscrete::EstimatorContinuousDiscrete()
    : EstimatorEKF()
    , xhat_(Eigen::VectorXf::Zero(13))
    , P_(Eigen::MatrixXf::Identity(13, 13))
    , Q_(Eigen::MatrixXf::Identity(13, 13))
    , Q_g_(Eigen::MatrixXf::Identity(6, 6))
    , R_(Eigen::MatrixXf::Zero(5, 5))
    , R_fast(Eigen::MatrixXf::Zero(2, 2))
{
  // This binds the various functions for the measurement and dynamic models and their jacobians,
  // to a reference that is efficient to pass to the functions used to do the estimation.
  bind_functions(); 

  load_magnetic_model();
  // Indicates whether inclination_ and declination_ have been updated.
  mag_init_ = false;
  // Initialize the baro low pass filter.
  lpf_static_ = 0.0;

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  // Initialize estimator from ROS2 parameters
  initialize_process_noises();
  initialize_state_covariances();
  update_measurement_model_parameters();
  
}

// ======== MAIN ESTIMATION LOOP ========
void EstimatorContinuousDiscrete::estimate(const Input & input, Output & output)
{

  prediction_step(input);

  // Measurement updates.
  fast_measurement_update_step(input);
  gnss_measurement_update_step(input);
  

  check_estimate(input);
  
  // Low pass filter gyros to estimate angular rates ASK: Should we lpf this?
  lpf_gyro_x_ = alpha_ * lpf_gyro_x_ + (1 - alpha_) * input.gyro_x;
  lpf_gyro_y_ = alpha_ * lpf_gyro_y_ + (1 - alpha_) * input.gyro_y;
  lpf_gyro_z_ = alpha_ * lpf_gyro_z_ + (1 - alpha_) * input.gyro_z;
  
  // Stuff the output
  output.pn = xhat_(0);
  output.pe = xhat_(1);
  output.pd = xhat_(2);
  output.vn = xhat_(3);
  output.ve = xhat_(4);
  output.vd = xhat_(5);
  output.p = lpf_gyro_x_;
  output.q = lpf_gyro_y_;
  output.r = lpf_gyro_z_;
  output.phi = xhat_(6);
  output.theta = xhat_(7);
  output.psi = xhat_(8);
  output.bx = xhat_(9);
  output.by = xhat_(10);
  output.bz = xhat_(11);
  output.inclination = xhat_(12);
}

// ======== ESTIMATION LOOP STEPS ========
void EstimatorContinuousDiscrete::prediction_step(const Input& input)
{
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;

  Eigen::Vector<float, 6> imu_measurements; // These are the input to the propagation.
  imu_measurements << input.accel_x, input.accel_y, input.accel_z,
                      input.gyro_x, input.gyro_y, input.gyro_z;

  std::tie(P_, xhat_) = propagate_model(xhat_, multirotor_dynamics_model, multirotor_jacobian_model,
                                        imu_measurements, multirotor_input_jacobian_model, P_, Q_,
                                        Q_g_, Ts);
  
  // Wrap RPY estimates.
  xhat_(6) = wrap_within_180(0.0, xhat_(6));
  xhat_(7) = wrap_within_180(0.0, xhat_(7));
  xhat_(8) = wrap_within_180(0.0, xhat_(8));
}

void EstimatorContinuousDiscrete::fast_measurement_update_step(const Input& input)
{
  calc_mag_field_properties(input); // Runs only once, finds inclination_ and declination_.
  
  // Only update when have new baro and new mag.
  if (!new_baro_) {
    return; // ASK: Should we split the magnetometer?
  }


  // bool convert_to_gauss = params_.get_bool("convert_to_gauss");

  lpf_static_ = alpha1_ * lpf_static_ + (1 - alpha1_) * input.static_pres; // ASK: Should we nix this?

  // Eigen::Vector3f mag_readings;
  // mag_readings << input.mag_x, input.mag_y, input.mag_z;
  // if (convert_to_gauss) {
  //   mag_readings *= 10'000.;
  // }
  
  Eigen::Vector<float, 2> y_fast;
  y_fast << lpf_static_, true_heading_;

  Eigen::Vector<float, 1> _;
  std::tie(P_, xhat_) = measurement_update(xhat_, _, multirotor_fast_measurement_model,
                                           y_fast, multirotor_fast_measurement_jacobian_model,
                                           R_fast, P_);

  new_baro_ = false;
}

void EstimatorContinuousDiscrete::gnss_measurement_update_step(const Input& input)
{
  Eigen::VectorXf _(1); // This is used when no inputs are needed.

  // Only update if new GPS information is available.
  if (input.gps_new) {
    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));
    gps_course = wrap_within_180(xhat_(3), gps_course);
    
    // Measurements for the positional states.
    Eigen::Vector<float, 5> y_pos;
    y_pos << input.gps_n, input.gps_e, input.gps_vn, input.gps_ve, input.gps_vd;
    
    auto h = multirotor_measurement_prediction(xhat_, _);
    auto C = multirotor_measurement_jacobian(xhat_, _);
    // Update the state and covariance with based on the predicted and actual measurements.
    std::tie(P_, xhat_) = measurement_update(xhat_, _, multirotor_measurement_model, y_pos,
                                             multirotor_measurement_jacobian_model, R_, P_);
  }
}

// ======== PREDICITON STEP EQUATIONS ========
// These are passed by reference to the predition step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{

  double gravity = params_.get_double("gravity");
  
  // Unpack states and inputs.
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  Eigen::Vector3f Theta = state.block<3,1>(6,0); // Theta is the vector of the euler angles.
  Eigen::Vector3f biases = state.block<3,1>(9,0);

  Eigen::Vector3f y_accel = inputs.block<3,1>(0,0);
  Eigen::Vector3f y_gyro = inputs.block<3,1>(3,0);
  
  // Calculate the derivatives of the states (see Chapter 14 of the UAVbook)
  Eigen::Vector3f velocity_dot = gravity*Eigen::Vector3f::UnitZ() + R(Theta)*y_accel;
  Eigen::Vector3f euler_angles_dot = S(Theta)*(y_gyro - biases);
  
  // Stuff the derivative vector with the respective derivatives.
  Eigen::Vector<float, 13> f = Eigen::Vector<float, 13>::Zero();
  f << vels, velocity_dot, euler_angles_dot;

  return f;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{
  Eigen::Vector3f accel = inputs.block<3,1>(0,0);
  Eigen::Vector3f gyro = inputs.block<3,1>(3,0);
  
  Eigen::Vector3f Theta = state.block<3,1>(6,0);
  Eigen::Vector3f biases = state.block<3,1>(9,0);

  Eigen::Matrix<float, 13, 13> A = Eigen::Matrix<float, 13, 13>::Zero();

  // Identity matrix.
  A.block<3,3>(0,3) = Eigen::Matrix3f::Identity();
  
  // del R(Theta)*y_accel / del Theta
  A.block<3,3>(3,6) = del_R_Theta_y_accel_del_Theta(Theta, accel);

  // del S(Theta)(y_gyro - b) / del Theta
  A.block<3,3>(6,6) = del_S_Theta_del_Theta(Theta, biases, gyro);

  A.block<3,3>(6,9) = -S(Theta);

  return A;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{
  // This uses both the accel and gyro. The associated jacobians have been combined.
  Eigen::Matrix<float, 13, 6> G = Eigen::Matrix<float, 13, 6>::Zero();
  
  Eigen::Vector3f Theta = state.block<3,1>(6,0);
  
  G.block<3,3>(3,0) = -R(Theta);
  G.block<3,3>(6,3) = -S(Theta);

  return G;
}

// ======== FAST MEAUREMENT STEP EQUATIONS========
// These are passed by reference to the fast measurement update step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_fast_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");

  // float declination = input(0);
  //
  // float inclination = xhat_(12);
  // 
  // Eigen::Vector3f Theta = state.block<3,1>(6,0);

  Eigen::VectorXf h = Eigen::VectorXf::Zero(2);

  // Eigen::Vector3f inertial_mag_readings = calculate_inertial_magnetic_field(declination, inclination);
  //
  // // Rotate the magnetometer readings into the body frame.
  // Eigen::Vector3f predicted_mag_readings = R(Theta)*inertial_mag_readings;
  // predicted_mag_readings /= predicted_mag_readings.norm();
  
  // Predicted static pressure measurements
  h(0) = -rho*gravity*state(2);

  // Predicted magnetometer measurement in each body axis.
  h(1) = state(8);

  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_fast_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");
  // 
  // Eigen::Vector3f Theta = state.block<3,1>(6,0);
  //
  // float declination = input(0);
  //
  // float inclination = -xhat_(12);
  //
  // Eigen::Matrix<float, 3, 4> R_theta_mag_jac = del_R_Theta_inc_y_mag_del_Theta(Theta, inclination, declination);

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(2,13);

  // Static pressure
  C(0,2) = -rho*gravity;
  C(1,8) = 1;
  
  // Magnetometer update
  // C.block<3,3>(1,6) = R_theta_mag_jac.block<3,3>(0,0);
  // C.block<3,1>(1,12) = R_theta_mag_jac.col(3);
  return C;
}

// ======== GNSS MEAUREMENT STEP EQUATIONS========
// These are passed by reference to the GNSS measurement update step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float v_n = state(3);
  float v_e = state(4);
  float v_d = state(5);
  
  Eigen::VectorXf h = Eigen::VectorXf::Zero(5);

  // North position
  h(0) = state(0);

  // East position
  h(1) = state(1);
  
  // Vel north
  h(2) = v_n;

  // Vel east
  h(3) = v_e;
  
  // Vel down
  h(4) = v_d;
  
  // To add a new measurement, simply use the state and any input you need as another entry to h. Be sure to update the measurement jacobian C.
   
  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(5,13);
  
  // GPS north
  C(0,0) = 1;

  // GPS east
  C(1,1) = 1;

  // GPS velocities
  C(2,3) = 1;
  C(3,4) = 1;
  C(4,5) = 1;

  // To add a new measurement use the inputs and the state to add another row to the matrix C. Be sure to update the measurment prediction vector h.

  return C;
}

// ======== MEASUREMENT UPDATE HELPER FUNCTIONS========
Eigen::Matrix3f EstimatorContinuousDiscrete::R(const Eigen::Vector3f& Theta)
{
  // Finds rotation matrix from the inertial frame to the body frame using
  // RPY angles in the vector Theta.
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  Eigen::Matrix3f R_theta;
  R_theta << cosf(psi)*cosf(theta), sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi), sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi),
             sinf(psi)*cosf(theta), sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi), - sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi),
             -sinf(theta), sinf(phi)*cosf(theta), cosf(phi)*cosf(theta);
  return R_theta;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::S(const Eigen::Vector3f& Theta)
{
  // Finds transition matrix using RPY angles on the vector Theta. See chapter 14 of the UAVbook.
  float phi = Theta(0);
  float theta = Theta(1);

  Eigen::Matrix3f S_theta;
  S_theta << 1.0, sinf(phi)*tanf(theta), cosf(phi)*tanf(theta),
             0.0, cosf(phi), -sinf(phi),
             0.0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta);
  return S_theta;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::del_S_Theta_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& biases,
                                                                   const Eigen::Vector3f& gyro)
{
  float bias_y = biases(1);
  float bias_z = biases(2);
  
  float gyro_y = gyro(1);
  float gyro_z = gyro(2);

  float phi = Theta(0);
  float theta = Theta(1);

  Eigen::Matrix3f S_Theta_jacobian;
  S_Theta_jacobian << 
  ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*tanf(theta),
  ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))/(cosf(theta)*cosf(theta)),
  0.0,
  (bias_y - gyro_y)*sinf(phi) + (bias_z - gyro_z)*cosf(phi),
  0.0,
  0.0,
  ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*(1.0/cosf(theta)), 
  ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))*tanf(theta)*(1.0/cosf(theta)), 
  0.0;

  return S_Theta_jacobian;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::del_R_Theta_y_accel_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& accel)
{
  float accel_x = accel(0);
  float accel_y = accel(1);
  float accel_z = accel(2);
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  Eigen::Matrix3f R_Theta_jacobian;
  R_Theta_jacobian <<
  accel_y*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) - accel_z *(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)),
  (- accel_x*sin(theta) + accel_y*sin(phi)*cos(theta) + accel_z*cos(phi)*cos(theta))*cos(psi),
  - accel_x*sinf(psi)*cosf(theta) - accel_y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + accel_z*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)),
  - accel_y*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)) - accel_z*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)),
  (- accel_x*sinf(theta) + accel_y*sinf(phi)*cosf(theta) + accel_z*cosf(phi)*cosf(theta))*sinf(psi), 
  accel_x*cosf(psi)*cosf(theta) + accel_y*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + accel_z*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)),
  (accel_y*cosf(phi) - accel_z*sinf(phi))*cosf(theta),
   - accel_x*cosf(theta) - accel_y*sinf(phi)*sinf(theta) - accel_z*sinf(theta)*cosf(phi),
  0.0;

  return R_Theta_jacobian;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::del_R_Theta_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& inertial_mag)
{
  float m_x = inertial_mag(0); // These are the mag measures in the inertial frame.
  float m_y = inertial_mag(1);
  float m_z = inertial_mag(2);

  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  Eigen::Matrix3f R_theta_mag_jac;
  R_theta_mag_jac << m_y*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) - m_z*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)),
  (- m_x*sinf(theta) + m_y*sinf(phi)*cosf(theta) + m_z*cosf(phi)*cosf(theta)) * cosf(psi),
  -m_x*sinf(psi)*cosf(theta) - m_y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + m_z*(sinf(phi)*cosf(psi)- sinf(psi)*sinf(theta)*cosf(phi)),
  -m_y*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)) - m_z*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)),
  (-m_x*sinf(theta) + m_y*sinf(phi)*cosf(theta) + m_z*cosf(phi)*cosf(theta))*sinf(psi),
  m_x*cosf(psi)*cosf(theta) + m_y*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + m_z*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)),
  (m_y*cosf(phi) - m_z*sinf(phi))*cosf(theta), - m_x*cosf(theta) - m_y*sinf(phi)*sinf(theta) - m_z*sinf(theta)*cosf(phi), 0;

  return R_theta_mag_jac;
}

Eigen::Matrix<float, 3,4> EstimatorContinuousDiscrete::del_R_Theta_inc_y_mag_del_Theta(const Eigen::Vector3f& Theta, const double& inclination,  const double& declination)
{
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  Eigen::Matrix<float, 3, 4> R_theta_inc_mag_jac;

R_theta_inc_mag_jac << -(sinf(psi) * cosf(declination + phi) - sinf(theta) * sinf(declination + phi) * cosf(psi)) * sinf(inclination), -(sinf(inclination) * cosf(theta) * cosf(declination + phi) + sinf(theta) * cosf(inclination)) * cosf(psi),(sinf(psi) * sinf(theta) * cosf(declination + phi) - sinf(declination + phi) * cosf(psi)) * sinf(inclination) -sinf(psi) * cosf(inclination) * cosf(theta), -(sinf(psi) * sinf(declination + phi) + sinf(theta) * cosf(psi) * cosf(declination + phi)) * cosf(inclination) -sinf(inclination) * cosf(psi) * cosf(theta),
(sinf(psi) * sinf(theta) * sinf(declination + phi) + cosf(psi) * cosf(declination + phi)) * sinf(inclination), -(sinf(inclination) * cosf(theta) * cosf(declination + phi) + sinf(theta) * cosf(inclination)) * sinf(psi), -(sinf(psi) * sinf(declination + phi)  + sinf(theta) * cosf(psi) * cosf(declination + phi)) * sinf(inclination) +cosf(inclination) * cosf(psi) * cosf(theta),(- sinf(psi) * sinf(theta) * cosf(declination + phi) + sinf(declination + phi) * cosf(psi)) * cosf(inclination) -sinf(inclination) * sinf(psi) * cosf(theta),
sinf(inclination) * sinf(declination + phi) * cosf(theta),sinf(inclination) * sinf(theta) * cosf(declination + phi) -cosf(inclination) * cosf(theta), 0,sinf(inclination) * sinf(theta) -cosf(inclination) * cosf (theta) * cosf(declination + phi);

  return R_theta_inc_mag_jac;
}

// ======== MISC HELPER FUNCTIONS========
Eigen::Vector3f EstimatorContinuousDiscrete::calculate_inertial_magnetic_field(const float& declination, const float& inclination)
{
  // The full intesity of the magnetic field is in the x axis in the magnetic frame.
  Eigen::Vector3f mag_x = Eigen::Vector3f::UnitX();

  Eigen::Matrix3f mag_inclination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(-inclination, Eigen::Vector3f::UnitY());
  
  Eigen::Matrix3f mag_declination_rotation;
  mag_declination_rotation = Eigen::AngleAxisf(declination, Eigen::Vector3f::UnitZ());
  
  // Find the magnetic field intenisty described in the inertial frame by rotating the frame.
  Eigen::Matrix3f mag_rotation = mag_declination_rotation*mag_inclination_rotation;
  Eigen::Vector3f inertial_mag_readings = mag_rotation*mag_x;

  return inertial_mag_readings/inertial_mag_readings.norm();
}


void EstimatorContinuousDiscrete::calc_mag_field_properties(const Input& input) // TODO: Put in GPS init?
{
  if (mag_init_ || !has_fix_ || !input.gps_new) {
    return;
  }

  double total_intensity; // nanoTesla (unused)
  double grid_variation; // Only useful for arctic or antarctic navigation (unused).
  
  // Take the current year and then add a decimal for the current day. USE GPS TIME.
  // This is a rough interpolation for speed. This will be accurate +- 1 day which is a time decimal change of ~0.0027
  // therefore this method isn't completely accurate. Other sources of error will be much larger.
  float decimal_month = input.gps_month + input.gps_day/31.0;
  float decimal_year = input.gps_year + decimal_month/12.0;
  
  int mag_success = geomag_calc(input.gps_alt/1000.0,
                                input.gps_lat,
                                input.gps_lon,
                                decimal_year,
                                &declination_,
                                &inclination_,
                                &total_intensity,
                                &grid_variation);

  xhat_(12) = radians(inclination_);

  if (mag_success == -1) {
    RCLCPP_ERROR(this->get_logger(), "Something went wrong while calculating inclination and declination.");
    RCLCPP_ERROR(this->get_logger(), "Inclination and Declination not set, estimation will likely be poor.");
  }
  else {
    mag_init_ = true;
  }
}

void EstimatorContinuousDiscrete::check_estimate(const Input& input)
{
  double gps_n_lim = params_.get_double("gps_n_lim");
  double gps_e_lim = params_.get_double("gps_e_lim");

  bool problem = false;
  int prob_index = 0;
  for (auto state : xhat_) {
    if (!std::isfinite(state)) {
      if (!problem) {
        problem = true;
      }
      switch (prob_index) {
        case 0:
          xhat_(0) = input.gps_n;
          break;
        case 1:
          xhat_(1) = input.gps_e;
          break;
        case 2:
          xhat_(2) = -input.gps_alt;
          break;
        case 3:
          xhat_(3) = input.gps_vn;
          break;
        case 4:
          xhat_(4) = input.gps_ve;
          break;
        case 5:
          xhat_(5) = input.gps_vd;
          break;
        default:
          xhat_(prob_index) = 0.0;
          break;
      }
      initialize_state_covariances();
    }
    prob_index++;
    if (problem) {
      RCLCPP_WARN(this->get_logger(), "Estimator reinitialized due to non-finite state %d",
                  prob_index);
      problem = false;
    }
  }

  if (xhat_(0) > gps_n_lim || xhat_(0) < -gps_n_lim) {
    RCLCPP_WARN(this->get_logger(), "gps n limit reached");
    xhat_(0) = input.gps_n;
  }
  if (xhat_(1) > gps_e_lim || xhat_(1) < -gps_e_lim) {
    RCLCPP_WARN(this->get_logger(), "gps e limit reached");
    xhat_(1) = input.gps_e;
  }
}

// ======== INITIALIZATION FUNCTIONS ========
void EstimatorContinuousDiscrete::initialize_state_covariances() 
{
  // TODO: convert to parameter array.
  double pos_n_initial_cov = params_.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params_.get_double("pos_e_initial_cov");
  double pos_d_initial_cov = params_.get_double("pos_d_initial_cov");
  double vn_initial_cov = params_.get_double("vn_initial_cov");
  double ve_initial_cov = params_.get_double("ve_initial_cov");
  double vd_initial_cov = params_.get_double("vd_initial_cov");
  double phi_initial_cov = params_.get_double("phi_initial_cov");
  double theta_initial_cov = params_.get_double("theta_initial_cov");
  double psi_initial_cov = params_.get_double("psi_initial_cov");
  double bias_x_initial_cov = params_.get_double("bias_x_initial_cov");
  double bias_y_initial_cov = params_.get_double("bias_y_initial_cov");
  double bias_z_initial_cov = params_.get_double("bias_z_initial_cov");
  double inclination_initial_cov = params_.get_double("inclination_initial_cov");
  
  P_ = Eigen::MatrixXf::Identity(13, 13); // TODO: make number of states a global variable so you only have to change it once.
  P_(0, 0) = pos_n_initial_cov;
  P_(1, 1) = pos_e_initial_cov;
  P_(2, 2) = pos_d_initial_cov;
  P_(3, 3) = vn_initial_cov;
  P_(4, 4) = ve_initial_cov;
  P_(5, 5) = vd_initial_cov;
  P_(6, 6) = phi_initial_cov; 
  P_(7, 7) = theta_initial_cov; 
  P_(8, 8) = psi_initial_cov; 
  P_(9, 9) = bias_x_initial_cov; 
  P_(10, 10) = bias_y_initial_cov; 
  P_(11, 11) = bias_z_initial_cov; 
  P_(12, 12) = inclination_initial_cov; 
}

void EstimatorContinuousDiscrete::initialize_process_noises() 
{
  double gyro_process_noise = params_.get_double("gyro_process_noise");
  double accel_process_noise = params_.get_double("accel_process_noise");
  double position_process_noise = params_.get_double("pos_process_noise");
  double roll_process_noise = params_.get_double("roll_process_noise");
  double pitch_process_noise = params_.get_double("pitch_process_noise");
  double yaw_process_noise = params_.get_double("yaw_process_noise");
  double alt_process_noise = params_.get_double("alt_process_noise");
  double velocity_process_noise = params_.get_double("vel_horizontal_process_noise");
  double velocity_vertical_process_noise = params_.get_double("vel_horizontal_process_noise");
  double bias_process_noise = params_.get_double("bias_process_noise");
  double inclination_process_noise = params_.get_double("inclination_process_noise");
  
  Eigen::VectorXf imu_process_noises = Eigen::VectorXf::Zero(6);
  imu_process_noises << pow(accel_process_noise,2), pow(accel_process_noise,2), pow(accel_process_noise,2),
                        pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2);

  Q_g_ = Eigen::DiagonalMatrix<float,6>(imu_process_noises);

  Q_(0,0) = position_process_noise;
  Q_(1,1) = position_process_noise;
  Q_(2,2) = alt_process_noise;
  Q_(3,3) = velocity_process_noise;
  Q_(4,4) = velocity_process_noise;
  Q_(5,5) = velocity_vertical_process_noise;
  Q_(6,6) = roll_process_noise;
  Q_(7,7) = pitch_process_noise;
  Q_(8,8) = yaw_process_noise;
  Q_(9,9) = bias_process_noise;
  Q_(10,10) = bias_process_noise;
  Q_(11,11) = bias_process_noise;
  Q_(12,12) = inclination_process_noise;
}

void EstimatorContinuousDiscrete::update_measurement_model_parameters()
{
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params_.get_double("sigma_n_gps");
  double sigma_e_gps = params_.get_double("sigma_e_gps");
  double sigma_vn_gps = params_.get_double("sigma_vn_gps");
  double sigma_ve_gps = params_.get_double("sigma_ve_gps");
  double sigma_vd_gps = params_.get_double("sigma_vd_gps");
  double sigma_static_press = params_.get_double("sigma_static_press");
  double sigma_mag = params_.get_double("sigma_mag");
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float lpf_a = params_.get_double("lpf_a");
  float lpf_a1 = params_.get_double("lpf_a1");

  R_(0, 0) = powf(sigma_n_gps, 2);
  R_(1, 1) = powf(sigma_e_gps, 2);
  R_(2, 2) = powf(sigma_vn_gps, 2);
  R_(3, 3) = powf(sigma_ve_gps, 2);
  R_(4, 4) = powf(sigma_vd_gps, 2);
  
  R_fast(0,0) = powf(sigma_static_press,2);
  R_fast(1,1) = powf(sigma_mag,2);
  
  // Calculate low pass filter alpha values.
  alpha_ = exp(-lpf_a * Ts);
  alpha1_ = exp(-lpf_a1 * Ts);
}

void EstimatorContinuousDiscrete::declare_parameters()
{ // TODO: Add a param file to pull from.
  
  // Sensor uncertainties
  params_.declare_double("sigma_n_gps", .01);
  params_.declare_double("sigma_e_gps", .01);
  params_.declare_double("sigma_vn_gps", .01);
  params_.declare_double("sigma_ve_gps", .01);
  params_.declare_double("sigma_vd_gps", .01);
  params_.declare_double("sigma_static_press", 1.0);
  params_.declare_double("sigma_mag", 1.0); 
  params_.declare_double("sigma_accel", .0025 * 9.81);

  // Low pass filter parameters
  params_.declare_double("lpf_a", 50.0);
  params_.declare_double("lpf_a1", 8.0);
  
  // Proccess noises
  params_.declare_double("roll_process_noise", 0.0000000001);
  params_.declare_double("pitch_process_noise", 0.000000000001);
  params_.declare_double("yaw_process_noise", 0.000000001);
  params_.declare_double("gyro_process_noise", 0.13);
  params_.declare_double("accel_process_noise", 024525);
  params_.declare_double("pos_process_noise", 0.0000001);
  params_.declare_double("alt_process_noise", 0.001);
  params_.declare_double("vel_horizontal_process_noise", 0.0001); 
  params_.declare_double("vel_vertical_process_noise", 0.00001);
  params_.declare_double("bias_process_noise", 0.00001);
  params_.declare_double("inclination_process_noise", 0.000000001);
  
  // Initial covariances
  params_.declare_double("pos_n_initial_cov", 100.);
  params_.declare_double("pos_e_initial_cov", 100.);
  params_.declare_double("pos_d_initial_cov", 100.);
  params_.declare_double("vn_initial_cov", 40.);
  params_.declare_double("ve_initial_cov", 40.);
  params_.declare_double("vd_initial_cov", 40.);
  params_.declare_double("phi_initial_cov", 0.005);
  params_.declare_double("theta_initial_cov", 0.005);
  params_.declare_double("psi_initial_cov", 0.01);
  params_.declare_double("bias_x_initial_cov", 0.0);
  params_.declare_double("bias_y_initial_cov", 0.0);
  params_.declare_double("bias_z_initial_cov", 0.0);
  params_.declare_double("inclination_initial_cov", 0.0001);
  
  // Conversion flags
  params_.declare_bool("convert_to_gauss", true);

  // Saturations limits
  params_.declare_double("max_estimated_phi", 85.0); // Deg
  params_.declare_double("max_estimated_theta", 80.0); // Deg
  params_.declare_double("gps_n_lim", 10000.);
  params_.declare_double("gps_e_lim", 10000.);
}

void EstimatorContinuousDiscrete::bind_functions()
{
  // Used to declutter the code.
  using namespace std::placeholders;
  using This = EstimatorContinuousDiscrete;

  // This creates references to the functions that are necessary estimate. This means we can pass them to the EKF class's functions.
  // std::bind creates a forwarding reference to a function. So when we pass the binding object to another method, that method can call the
  // original function.
  multirotor_dynamics_model = std::bind(&This::multirotor_dynamics, this, _1, _2);
  multirotor_jacobian_model = std::bind(&This::multirotor_jacobian, this, _1, _2);
  multirotor_input_jacobian_model = std::bind(&This::multirotor_input_jacobian, this, _1, _2);

  multirotor_measurement_model = std::bind(&This::multirotor_measurement_prediction, this, _1, _2);
  multirotor_measurement_jacobian_model = std::bind(&This::multirotor_measurement_jacobian, this, _1, _2);

  multirotor_fast_measurement_model = std::bind(&This::multirotor_fast_measurement_prediction, this, _1, _2);
  multirotor_fast_measurement_jacobian_model = std::bind(&This::multirotor_fast_measurement_jacobian, this, _1, _2);

}

} // namespace roscopter
