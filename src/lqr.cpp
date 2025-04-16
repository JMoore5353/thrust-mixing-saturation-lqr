#include "lqr.hpp"

namespace lqr_controller {

LqrController::LqrController()
  : rclcpp::Node("lqr_controller")
{
  // Declare parameters
  declare_parameters();
  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LqrController::parameters_callback, this, std::placeholders::_1));

  // Initialize ROS2 interfaces
  command_sub_ = this->create_subscription<rosflight_msgs::msg::Command>(
    "command", 10, std::bind(&LqrController::command_callback, this, std::placeholders::_1));
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
    "estimated_state", 10, std::bind(&LqrController::state_callback, this, std::placeholders::_1));
  pwm_sub_ = this->create_subscription<rosflight_msgs::msg::OutputRaw>(
    "output_raw", 1, std::bind(&LqrController::pwm_callback, this, std::placeholders::_1));
  forces_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "forces_and_moments", 1, std::bind(&LqrController::forces_callback, this, std::placeholders::_1));
  command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>(
    "command_lqr", 1);

  // Create the F_ matrix based off the rotor geometry
  F_ << -1, -1, 1, 1, 1, -1, -1, 1,
       compute_rotor_torque_constant(0, 0.0), // Initial f_des = 0.0
      -compute_rotor_torque_constant(1, 0.0),
       compute_rotor_torque_constant(2, 0.0),
      -compute_rotor_torque_constant(3, 0.0),
      1, 1, 1, 1;
  F_.row(0) *= sqrt(2) / 2 * this->get_parameter("arm_length").as_double();
  F_.row(1) *= sqrt(2) / 2 * this->get_parameter("arm_length").as_double();

  // Load the inertia matrix
  J_ << 0.08, 0, 0, 0, 0.08, 0, 0, 0, 0.12;

  compute_max_thrust();

  // Compute initial gains
  compute_lqr_gain();
}

void LqrController::declare_parameters()
{
  this->declare_parameter("mass", 3.5);
  this->declare_parameter("Q", std::vector<float>{150.0, 150.0, 100.0, 10.0, 10.0, 10.0});
  this->declare_parameter("R", std::vector<float>{10.0, 10.0, 10.0});
  this->declare_parameter("alpha_up", 0.011); // 11ms from the paper
  this->declare_parameter("alpha_down", 0.027); // 27ms from the paper
  this->declare_parameter("arm_length", 0.30); // Distance from the motor to the center of gravity
  this->declare_parameter("print_debug", true);
  this->declare_parameter("CT", 0.08);
  this->declare_parameter("CQ", 0.0043);
  this->declare_parameter("D", 0.381);
  this->declare_parameter("rho", 1.225);
  this->declare_parameter("KV", 0.02894);
  this->declare_parameter("V_max", 25.0);
  this->declare_parameter("i0", 1.01);
  this->declare_parameter("motor_resistance", 0.085);
  this->declare_parameter("gravity", 9.81);
  this->declare_parameter("max_iters", 3);
  this->declare_parameter("iter_threshold", 0.01);
  this->declare_parameter("num_motors", 4);
  this->declare_parameter("use_throttle", false);
  this->declare_parameter("f_max_limit", 0.9);  // Percentage of max thrust
  this->declare_parameter("f_min_limit", 0.01);  // Percentage of max thrust
  this->declare_parameter("assured_yaw_torque", 0.2); // Nm
  this->declare_parameter("use_prioritized_input_sat", true);
}

rcl_interfaces::msg::SetParametersResult LqrController::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (auto param : parameters) {
    if (param.get_name() == "Q") {
      if (param.as_double_array().size() != 6) {
        result.successful = false;
        result.reason = "Q must be a 6 vector! Not setting parameter.";
        return result;
      }

      for (auto val : param.as_double_array()) {
        if (val < 0) {
          result.successful = false;
          result.reason = "Q must be positive semi-definite! Not setting parameter.";
          return result;
        }
      }
    } else if (param.get_name() == "R") {
      if (param.as_double_array().size() != 3) {
        result.successful = false;
        result.reason = "R must be a 3 vector! Not setting parameter.";
        return result;
      }

      for (auto val : param.as_double_array()) {
        if (val <= 0) {
          result.successful = false;
          result.reason = "R must be positive definite! Not setting parameter.";
          return result;
        }
      }
    }
  }

  // Set the flag to recompute controller gains
  controller_parameters_changed_ = true;

  return result;
} 

void LqrController::compute_max_thrust()
{
  double CT = this->get_parameter("CT").as_double();
  double CQ = this->get_parameter("CQ").as_double();
  double D = this->get_parameter("D").as_double();
  double rho = this->get_parameter("rho").as_double();
  double KV = this->get_parameter("KV").as_double();
  double V_max = this->get_parameter("V_max").as_double();
  double i0 = this->get_parameter("i0").as_double();
  double Res = this->get_parameter("motor_resistance").as_double();
  double num_motors = this->get_parameter("num_motors").as_int();

  double a = Res * rho * pow(D, 5.0) * CQ / (4 * M_PI * M_PI * KV);
  double b = KV;
  double c = i0 * Res - V_max;
  double omega = (-b + sqrt(pow(b, 2.0) - 4*a*c)) / (2*a);
  max_thrust_ = rho * pow(D, 4.0) * CT * pow(omega, 2.0) / (4 * M_PI * M_PI) * num_motors;

  f_max_ = max_thrust_ / num_motors * this->get_parameter("f_max_limit").as_double();
  f_min_ = max_thrust_ / num_motors * this->get_parameter("f_min_limit").as_double();
}

void LqrController::compute_lqr_gain()
{
  // Form the system matrices
  double alpha_approx = (this->get_parameter("alpha_up").as_double() + this->get_parameter("alpha_down").as_double()) / 2;
  double val = 1 / alpha_approx;
  Eigen::DiagonalMatrix<double, 3, 3> eye_alpha(val, val, val);

  A_.topLeftCorner(6,3) = Eigen::Matrix<double, 6, 3>::Zero();
  A_.topRightCorner(3,3) = J_.inverse();
  A_.bottomRightCorner(3,3) = -1*eye_alpha;
  
  B_.topLeftCorner(3,3) = Eigen::Matrix3d::Zero();
  B_.bottomLeftCorner(3,3) = eye_alpha;

  // Construct the Hamiltonians and the gain matrix
  // Form the gain matrices
  Q_ = Eigen::Matrix<double, 6, 1>(this->get_parameter("Q").as_double_array().data()).asDiagonal();
  R_ = Eigen::Vector3d(this->get_parameter("R").as_double_array().data()).asDiagonal();

  // Form the Hamiltonian
  H_.topLeftCorner(6,6) = A_;
  H_.topRightCorner(6,6) = -1*B_ * R_.inverse() * B_.transpose();
  H_.bottomLeftCorner(6,6) = -1*Q_;
  H_.bottomRightCorner(6,6) = -1*A_.transpose();

  // Find the eigenvalues of the Hamiltonian
  Eigen::EigenSolver<Eigen::Matrix<double, 12, 12>> es(H_);
  Eigen::Matrix<std::complex<double>, 12, 1> eigvals = es.eigenvalues();
  Eigen::Matrix<std::complex<double>, 12, 12> eigvects = es.eigenvectors();

  // Find the stable eigenvalues and extract the eigenvectors
  int col_pointer = 0;
  Eigen::Matrix<double, 12, 6> V_stab;
  for (int i=0; i<12; ++i) {
    if (eigvals(i).real() < 0) {
      // There should be exactly 6 stable eigvals
      V_stab.col(col_pointer) = eigvects.col(i).real();
      col_pointer += 1;
    }
  }

  // if (this->get_parameter("print_debug").as_bool()) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "V_stab" << std::endl << V_stab);
  // }

  // Compute P (solution to ARE)!
  Eigen::Matrix<double, 6, 6> P;
  P = V_stab.bottomLeftCorner(6,6) * V_stab.topLeftCorner(6,6).inverse();

  // if (this->get_parameter("print_debug").as_bool()) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "P" << std::endl << P);
  // }

  // Compute LQR gain
  K_LQR_ = R_.inverse() * B_.transpose() * P;

  RCLCPP_INFO_STREAM(this->get_logger(), "New K gains:" << std::endl << K_LQR_);
}

void LqrController::state_callback(const roscopter_msgs::msg::State & msg)
{
  current_state_ = msg;
}

void LqrController::pwm_callback(const rosflight_msgs::msg::OutputRaw & msg)
{
  current_pwms_ = msg;
}

void LqrController::forces_callback(const geometry_msgs::msg::TwistStamped & msg)
{
  current_forces_ = msg;
}

void LqrController::command_callback(const rosflight_msgs::msg::Command & msg)
{
  if (msg.mode != rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "LQR controller takes in rate commands! passing message on to firmware!");
    command_pub_->publish(msg);
    return;
  }

  // Check to see if the ROS2 parameters have changed (if we need to recompute K_LQR_)
  if (controller_parameters_changed_) {
    compute_max_thrust();
    compute_lqr_gain();
    controller_parameters_changed_ = false;
  }

  // Parse input command
  Eigen::Vector3d omega_des(msg.qx, msg.qy, msg.qz);

  double c_des = 0.0;
  if (this->get_parameter("use_throttle").as_bool()) {
    double throttle_des = msg.fz; // Throttle
    c_des = throttle_des * max_thrust_ / this->get_parameter("mass").as_double(); // Mass normalized thrust
  } else {
    c_des = msg.fz / this->get_parameter("mass").as_double(); // Msg contains total desired thrust
  }

  // std::cout << "omega_des" << std::endl << omega_des << std::endl;

  // Estimate f_i and omega
  estimate_lqr_state();
  // std::cout << "Eta hat: " << eta_hat_ << std::endl;

  // Compute LQR control, n_des
  Eigen::Vector3d n_des = compute_n_des(omega_des);

  // std::cout << n_des << std::endl;

  // Iterative mixer for the first pass
  Eigen::Vector4d f_des = iterative_thrust_mixing(c_des, n_des);
  // std::cout << "f_des: " << std::endl << f_des_ << std::endl;
  // std::cout << "_______________" << std::endl;

  // Prioritized input saturation
  f_des_ = input_saturation(f_des, n_des, c_des);

  // Send final prioritized command to the firmware
  publish_command();
}

void LqrController::estimate_lqr_state()
{
  // Estimate omega
  omega_hat_(0) = current_state_.p;
  omega_hat_(1) = current_state_.q;
  omega_hat_(2) = current_state_.r;

  // Estimate eta_hat
  eta_hat_(0) = current_forces_.twist.angular.x;
  eta_hat_(1) = current_forces_.twist.angular.y;
  eta_hat_(2) = current_forces_.twist.angular.z;

  // Estimate the body torqes from the pwm signals sent to the motors
  // double rho = this->get_parameter("rho").as_double();
  // double CT = this->get_parameter("CT").as_double();
  // double CQ = this->get_parameter("CQ").as_double();
  // double Res = this->get_parameter("motor_resistance").as_double();
  // double KV = this->get_parameter("KV").as_double();
  // double D = this->get_parameter("D").as_double();
  // double i0 = this->get_parameter("i0").as_double();
  // double vmax = this->get_parameter("V_max").as_double();
  // for (int i=0; i<4; ++i) {
  //   // Eq 4.22 Small Unmanned Aircraft
  //   double vin = vmax * current_pwms_.values[i];
  //
  //   double a = -rho * pow(D, 5.0) * CQ / (4*M_PI*M_PI);
  //   double b = -KV*KV/Res;
  //   double c = KV*vin/Res - KV*i0;
  //   double omega_p = (-b - sqrt(pow(b,2.0) - 4*a*c)) / (2*a);
  //
  //   f_hat_(i) = CT*rho*pow(D, 4.0) / (4*M_PI*M_PI) * pow(omega_p, 2.0);
  // }

  // TODO: This is blowing up... How else do we estimate eta_hat?
  // Estimate body torques produced by the motors
  // double alpha = 0;
  // double now = 1e-9 * static_cast<double>(this->get_clock()->now().nanoseconds());
  // double dt = compute_dt(now);
  // std::cout << "DT: " << dt << std::endl; 
  // for (int i=0; i<4; ++i) {
  //   // Determine which alpha to use
  //   if (f_des_(i) >= f_hat_(i)) {
  //     alpha = this->get_parameter("alpha_up").as_double();
  //   } else {
  //     alpha = this->get_parameter("alpha_down").as_double();
  //   }
  //
  //   f_hat_(i) = f_des_(i) + (f_hat_(i) - f_des_(i)) * exp(-1 / alpha * dt);
  // }
  //
  //
  // // Compute eta from Eq. 1
  // // TODO: Could parametrize this in terms of location of motors/direction of motors
  // Eigen::Vector4d new_kappas(-compute_rotor_torque_constant(0, f_hat_(0)),
  //                             compute_rotor_torque_constant(1, f_hat_(1)),
  //                            -compute_rotor_torque_constant(2, f_hat_(2)),
  //                             compute_rotor_torque_constant(3, f_hat_(3)));
  // F_.row(2) = new_kappas;
  // eta_hat_ = F_.block(0,0,3,3) * f_hat_;
}

double LqrController::compute_rotor_torque_constant(int idx, double f)
{
  // Since I don't have access to torque and thrust data, compute it using the
  // equations from Ch4 of Small Unmanned Aircraft
  // Use equation 4.17 to compute omega_p, then use Eq 4.18 to compute Qp
  // WE DONT HAVE VA -- assume J constant ?
  // a = rho * pow(D, 4.0) * CT0 / (4 * M_PI * M_PI);
  // b = rho * pow(D, 3.0) * CT1 * Va / (2 * M_PI);
  // c = rho * pow(D, 2.0) * CT2 * pow(Va, 2.0) - f_hat_(idx);
  // omega_p = (-b + sqrt(pow(b, 2.0) - 4 * a * c)) / 2 / a;

  // One option is to use approximate values from the graphs from the paper
  // From ref[6]
  // double kappa = 0.0195;

  // Another option: Assume constant kappa -- k ~ 0.0.2047
  double D = this->get_parameter("D").as_double();
  double CQ = this->get_parameter("CQ").as_double();
  double CT = this->get_parameter("CT").as_double();
  double kappa = D * CQ / CT;
  return kappa;
}

Eigen::Vector3d LqrController::compute_n_des(Eigen::Vector3d omega_des)
{
  // Eq 6
  Eigen::Vector3d eta_ref = omega_des.cross(J_ * omega_des) + J_ * omega_dot_des_;

  // Form state vector (Eq. 12)
  Eigen::Matrix<double, 6, 1> error_state;
  error_state << omega_des - omega_hat_, eta_ref - eta_hat_;

  // Compute desired body torques (Eq. 12)
  Eigen::Vector3d n_des = K_LQR_ * error_state + omega_hat_.cross(J_ * omega_hat_) + J_ * omega_dot_des_;
  return n_des;
}

Eigen::Vector4d LqrController::iterative_thrust_mixing(double c_des, Eigen::Vector3d eta_des)
{
  // Initialize the solver
  double mass = this->get_parameter("mass").as_double();
  double initial_val = mass * c_des / 4.0;
  Eigen::Vector4d f_tmp(initial_val, initial_val, initial_val, initial_val);

  // Iterate until convergence
  double previous_norm = -100.0;
  double threshold = this->get_parameter("iter_threshold").as_double();
  int iteration_count = 0;
  while (f_tmp.norm() - previous_norm > threshold) {
    previous_norm = f_tmp.norm();

    // Compute k_i for each rotor
    for (int i=0; i<4; ++i) {
      double kappa = compute_rotor_torque_constant(i, f_tmp(i));
      // Update the A matrix
      F_(2, i) = kappa * pow(-1, i);
    }

    // Solve the equations for f_i
    Eigen::Vector4d b;
    b << eta_des, c_des * mass;
    // std::cout << "B" << std::endl << b << std::endl;
    f_tmp = F_.colPivHouseholderQr().solve(b);

    // Protect against eternal loops
    iteration_count += 1;
    if (iteration_count >= this->get_parameter("max_iters").as_int()) {
      RCLCPP_WARN_STREAM(this->get_logger(),
        "Warning: max interation count exceeded! Diff is " << f_tmp.norm() - previous_norm);
      break;
    }
  }

  return f_tmp;
}

int LqrController::find_biggest_violation(Eigen::Vector4d f_des)
{
  int current_max = -1;
  double current_error = 0.0;
  for (int i=0; i<4; ++i) {
    if (f_des(i) > f_max_) {
      double error = abs(f_des(i) - f_max_);
      if (current_max == -1 || error > current_error) {
        current_max = i;
        current_error = error;
      }
    } else if (f_des(i) < f_min_) {
      double error = abs(f_des(i) - f_min_);
      if (current_max == -1 || error > current_error) {
        current_max = i;
        current_error = error;
      }
    }
  }

  return current_max;
}

bool LqrController::check_both_limits(Eigen::Vector4d f_des)
{
  bool upper_limit_reached = false;
  bool lower_limit_reached = false;
  for (int i=0; i<4; ++i) {
    if (f_des(i) > f_max_) {
      upper_limit_reached = true;
    } else if (f_des(i) < f_min_) {
      lower_limit_reached = true;
    }
  }

  return upper_limit_reached && lower_limit_reached;
}

Eigen::Vector4d LqrController::collective_thrust_saturation(Eigen::Vector4d f_des)
{
  int current_max = find_biggest_violation(f_des);
  if (current_max == -1) {
    return f_des;
  }

  // Not possible to fix by collective saturation if true
  if (check_both_limits(f_des)) { 
    return f_des;
  }

  double error = 0.0;
  if (f_des(current_max) < f_min_) {
    error = f_des(current_max) - f_min_;
  } else {
    error = f_max_ - f_des(current_max);
  }

  f_des = f_des.array() - error; // Shift all by the same amount
  return f_des;
}

Eigen::Vector4d LqrController::yaw_saturation(Eigen::Vector4d f_des, Eigen::Vector3d eta_des, double c_des, int current_max)
{
  // Check to see if we are below n_assured
  // TODO: Change to a percentage of commanded yaw torque
  // if (abs(eta_des(2)) <= this->get_parameter("assured_yaw_torque").as_double()) {
  //   return f_des;
  // }

  double f_i = f_max_;
  if (f_des(current_max) < f_min_) { f_i = f_min_; }

  // Set up solver
  Eigen::Vector4d b;
  Eigen::Matrix4d A = F_;
  b << eta_des, c_des * this->get_parameter("mass").as_double();

  // TODO: Iterative thrust mixing
  b = b - f_i * A.col(current_max);  // Move ith col over to the other side of Ax = b
  A.col(current_max) = Eigen::Vector4d::Zero();  // Subtract that same col from the col of F
  A(2, current_max) = 1; // Set the (2,i) component to be 1
  b(2) -= eta_des(2); // Subtract eta_z from b (i.e. move it to the other side)

  // Solve
  Eigen::Vector4d new_vals = A.colPivHouseholderQr().solve(b);

  // Reinterpret values from new_vals
  for (int i=0; i<4; ++i) {
    if (i == current_max) {
      f_des(i) = f_i;
      eta_des(2) = new_vals(i);
    } else {
      f_des(i) = new_vals(i);
    }
  }

  // TODO: Check if we pass n_assured

  return f_des;
}

Eigen::Vector4d LqrController::input_saturation(Eigen::Vector4d f_des, Eigen::Vector3d eta_des, double c_des)
{
  std::cout << "INPUT SATURATION" << std::endl;
  std::cout << "f_des: " << f_des.transpose() << std::endl;

  if (this->get_parameter("use_prioritized_input_sat").as_bool()) {
    // If no motors were saturated, return f_des.
    int current_max = find_biggest_violation(f_des);
    if (current_max == -1) { return f_des; }

    std::cout << "Motor that is too big: " << current_max << std::endl;
    std::cout << f_des.transpose() << std::endl;

    // Yaw torque saturation
    f_des = yaw_saturation(f_des, eta_des, c_des, current_max);

    std::cout << "after yaw sat" << std::endl;
    std::cout << f_des.transpose() << std::endl;

    // Collective thrust saturation
    f_des = collective_thrust_saturation(f_des);

    std::cout << "after thrust sat" << std::endl;
    std::cout << f_des.transpose() << std::endl;
  }

  // Clipping
  for (int i=0; i<4; ++i) {
    f_des(i) = std::min(f_des(i), f_max_);
    f_des(i) = std::max(f_des(i), f_min_);
  }

  std::cout << "after clipping" << std::endl;
  std::cout << f_des.transpose() << std::endl;
  std::cout << "__________________" << std::endl;

  return f_des;
}

void LqrController::publish_command()
{
  // This assumes that the mixer in the firmware is set to use
  // USE_MOTOR_PARAM=1, with a custom mixer that looks like the 
  // identity matrix.
  rosflight_msgs::msg::Command out_msg;

  out_msg.header.stamp = this->get_clock()->now();
  out_msg.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
  out_msg.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;

  Eigen::Vector4d des_omega_sq = force_to_omega_squared();
  out_msg.fx = des_omega_sq(0);
  out_msg.fy = des_omega_sq(1);
  out_msg.fz = des_omega_sq(2);
  out_msg.qx = des_omega_sq(3);

  command_pub_->publish(out_msg);

  // Save the previous time
  prev_time_ = 1e-9 * static_cast<double>(this->get_clock()->now().nanoseconds());
}

double LqrController::compute_dt(double now)
{
  if(prev_time_ == 0) {
    prev_time_ = now;
    return 0;
  }

  // Calculate time
  double dt = now - prev_time_;
  prev_time_ = now;

  return dt;
}

Eigen::Vector4d LqrController::force_to_omega_squared()
{
  double rho = this->get_parameter("rho").as_double();
  double D = this->get_parameter("D").as_double();
  double CT = this->get_parameter("CT").as_double();
  double V_max = this->get_parameter("V_max").as_double();
  double KV = this->get_parameter("KV").as_double();
  double CQ = this->get_parameter("CQ").as_double();
  double Res = this->get_parameter("motor_resistance").as_double();
  double i0 = this->get_parameter("i0").as_double();

  // Equation from Chap 4 of Small Unmanned Aircraft
  Eigen::Vector4d out;
  for (int i=0; i<4; ++i) {
    double omega_squared = 4 * M_PI * M_PI * f_des_(i) / (rho * pow(D, 4.0) * CT);

    if (omega_squared < 0.0) {
      omega_squared = 0.0;
    }

    // Ch. 4, setting equation for torque produced by a propeller equal to Eq. 4.19
    // Note that we assume constant advance ratio, leading to constant torque and thrust constants.
    float V_in = rho * pow(D, 5.0) / (4.0 * pow(M_PI, 2.0)) * omega_squared * CQ * Res / KV
      + Res * i0 + KV * sqrt(omega_squared);

    // Convert desired V_in setting to a throttle setting
    out(i) = V_in / V_max;
  }

  return out;
}

} // namespace lqr_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lqr_controller::LqrController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
