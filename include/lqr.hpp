#include <cmath>
#include <complex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>

#include "rosflight_msgs/msg/command.hpp"
#include "roscopter_msgs/msg/state.hpp"
#include "rosflight_msgs/msg/output_raw.hpp"

namespace lqr_controller {

class LqrController : public rclcpp::Node {
public:
  LqrController();

private:
  // ROS2 interfaces
  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::OutputRaw>::SharedPtr pwm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr forces_sub_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  void declare_parameters();
  void state_callback(const roscopter_msgs::msg::State & msg);
  void pwm_callback(const rosflight_msgs::msg::OutputRaw & msg);
  void forces_callback(const geometry_msgs::msg::TwistStamped & msg);
  void command_callback(const rosflight_msgs::msg::Command & msg);
  void publish_command();
  /**
   * @brief Callback function to handle parameter changes
   * 
   * @param parameters: Vector of rclcpp::Parameter objects that were changed
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);


  void compute_max_thrust();
  void compute_lqr_gain();
  void estimate_lqr_state();
  double compute_rotor_torque_constant(int idx, double f);
  Eigen::Vector3d compute_n_des(Eigen::Vector3d omega_des);
  Eigen::Vector4d iterative_thrust_mixing(double c_des, Eigen::Vector3d eta_des);
  double compute_dt(double now);
  Eigen::Vector4d force_to_omega_squared();

  // Persistent variables
  roscopter_msgs::msg::State current_state_;
  rosflight_msgs::msg::OutputRaw current_pwms_;
  geometry_msgs::msg::TwistStamped current_forces_;
  Eigen::Vector3d omega_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d f_hat_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d f_des_ = Eigen::Vector4d::Zero();
  Eigen::Vector3d eta_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_dot_des_ = Eigen::Vector3d::Zero();
  double max_thrust_ = 0.0;

  // Matrices describing rotor geometry and inertia parameters
  Eigen::Matrix4d F_ = Eigen::Matrix4d::Zero();
  Eigen::Matrix3d J_ = Eigen::Matrix3d::Zero();

  // LQR Gain and system matrices
  Eigen::Matrix<double, 6, 6> A_;
  Eigen::Matrix<double, 6, 3> B_;
  Eigen::DiagonalMatrix<double, 6> Q_;
  Eigen::DiagonalMatrix<double, 3> R_;
  Eigen::Matrix<double, 12, 12> H_;
  Eigen::Matrix<double, 3, 6> K_LQR_;

  // Time variables
  double prev_time_ = 0;

  // Flag
  bool controller_parameters_changed_ = false;
};


} // namespace lqr_controller
