#ifndef ESTIMATOR_CONTINUOUS_DISCRETE_H
#define ESTIMATOR_CONTINUOUS_DISCRETE_H

#include <math.h>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "geomag.h"

#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"

namespace roscopter
{

class EstimatorContinuousDiscrete : public EstimatorEKF
{
public:
  EstimatorContinuousDiscrete();
  EstimatorContinuousDiscrete(bool use_params);

  Eigen::MatrixXf get_Q_(){return Q_;};
  Eigen::MatrixXf get_Q_g_(){return Q_g_;};
  Eigen::MatrixXf get_P_(){return P_;};
  Eigen::MatrixXf get_xhat_(){return xhat_;};
  auto get_dynamics(){return multirotor_dynamics_model;};
  auto get_jacobian(){return multirotor_jacobian_model;};
  auto get_input_jacobian(){return multirotor_input_jacobian_model;};

protected:
  virtual void estimate(const Input & input, Output & output);
private:

  double lpf_a_;
  float alpha_;
  float alpha1_;
  int N_;

  float lpf_gyro_x_;
  float lpf_gyro_y_;
  float lpf_gyro_z_;
  float lpf_static_;
  float lpf_diff_;
  float lpf_accel_x_;
  float lpf_accel_y_;
  float lpf_accel_z_;

  float phat_;
  float qhat_;
  float rhat_;
  float Vwhat_;
  float phihat_;
  float thetahat_;
  float psihat_;     // TODO: link to an inital condiditons param
  
  Eigen::VectorXf multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& measurements);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_dynamics_model;

  Eigen::MatrixXf multirotor_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& measurements);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> multirotor_jacobian_model;

  Eigen::MatrixXf multirotor_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> multirotor_input_jacobian_model;

  Eigen::VectorXf multirotor_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_measurement_model;
  
  Eigen::VectorXf multirotor_fast_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_fast_measurement_model;
  
  Eigen::MatrixXf multirotor_fast_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_fast_measurement_jacobian_model;

  Eigen::MatrixXf multirotor_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_measurement_jacobian_model;

  Eigen::VectorXf xhat_; // 12
  Eigen::MatrixXf P_;    // 12x12

  Eigen::MatrixXf Q_; // 12x12 // FIXME: fix the annotated sizes.
  Eigen::MatrixXf Q_g_; // 7x7
  Eigen::MatrixXf R_; // 6x6
  Eigen::MatrixXf R_fast; // 6x6

  // TODO: not used
  float gate_threshold_ = 9.21; // chi2(q = .01, df = 2)

  void bind_functions();

  /**
   * @brief This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  /**
   * @brief Initializes some variables that depend on ROS2 parameters
  */
  void update_measurement_model_parameters();

  /**
   * @brief Initializes the covariance matrices and process noise matrices with the ROS2 parameters
   */
  void initialize_uncertainties();

  /**
   * @brief Initializes the state covariance matrix with the ROS2 parameters
   */
  void initialize_state_covariances();
}; 

} // namespace roscopter

#endif // ESTIMATOR_CONTINUOUS_DISCRETE_H
