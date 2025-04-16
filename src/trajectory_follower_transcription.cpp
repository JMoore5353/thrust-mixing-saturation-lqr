#include <rclcpp/rclcpp.hpp>

#include "roscopter_msgs/msg/controller_command.hpp"
#include "rosflight_msgs/msg/command.hpp"
#include "roscopter_msgs/msg/state.hpp"

#include "simple_pid.hpp"

namespace lqr_controller {

class TrajectoryFollowerTranscription : public rclcpp::Node {
public:
  // Constructor
  TrajectoryFollowerTranscription() : Node("trajectory_transcription") {
    declare_parameters();
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryFollowerTranscription::parameters_callback, this, std::placeholders::_1));

    traj_follower_sub_ = this->create_subscription<roscopter_msgs::msg::ControllerCommand>(
      "high_level_command", 1, std::bind(&TrajectoryFollowerTranscription::traj_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
      "estimated_state", 1, std::bind(&TrajectoryFollowerTranscription::state_callback, this, std::placeholders::_1));
    command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 1);

    double roll_p = this->get_parameter("roll_p").as_double();
    double roll_d = this->get_parameter("roll_d").as_double();
    roll_pid_ = roscopter::SimplePID(roll_p, 0.0, roll_d);
    pitch_pid_ = roscopter::SimplePID(roll_p, 0.0, roll_d);

    double yaw_p = this->get_parameter("yaw_p").as_double();
    double yaw_d = this->get_parameter("yaw_d").as_double();
    yaw_pid_ = roscopter::SimplePID(yaw_p, 0.0, yaw_d);
  }


private:
  // ROS2 interfaces
  rclcpp::Subscription<roscopter_msgs::msg::ControllerCommand>::SharedPtr traj_follower_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  roscopter::SimplePID roll_pid_;
  roscopter::SimplePID pitch_pid_;
  roscopter::SimplePID yaw_pid_;
  bool do_reset_gains_ = false;
  roscopter_msgs::msg::State current_state_;

  void declare_parameters() {
    this->declare_parameter("roll_p", 5.0);
    this->declare_parameter("roll_d", 0.1);
    this->declare_parameter("yaw_p", 3.0);
    this->declare_parameter("yaw_d", 0.1);
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    do_reset_gains_ = true;
    return result;
  }

  void set_gains() {
    double roll_p = this->get_parameter("roll_p").as_double();
    double roll_d = this->get_parameter("roll_d").as_double();
    roll_pid_.set_gains(roll_p, 0.0, roll_d);
    pitch_pid_.set_gains(roll_p, 0.0, roll_d);

    double yaw_p = this->get_parameter("yaw_p").as_double();
    double yaw_d = this->get_parameter("yaw_d").as_double();
    yaw_pid_.set_gains(yaw_p, 0.0, yaw_d);
  }

  void state_callback(const roscopter_msgs::msg::State & msg) {
    current_state_ = msg;
  }

  void traj_callback(const roscopter_msgs::msg::ControllerCommand & msg) {
    // Reset the controller gains if needed
    if (do_reset_gains_) { set_gains(); }

    double phi = msg.cmd1;
    double theta = msg.cmd2;
    double psi = msg.cmd3;
    double thrust = msg.cmd4;

    // Compute PID from angles to rates
    rosflight_msgs::msg::Command out;

    double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    double dt = compute_dt(now);

    out.qx = roll_pid_.compute_pid(phi, current_state_.phi, dt, current_state_.p);
    out.qy = pitch_pid_.compute_pid(theta, current_state_.theta, dt, current_state_.q);
    out.qz = yaw_pid_.compute_pid(psi, current_state_.psi, dt, current_state_.r);
    out.fz = thrust;

    out.header.stamp = this->get_clock()->now();
    out.mode = rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;

    command_pub_->publish(out);
  }

  double compute_dt(double now)
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
};

} // namespace lqr_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lqr_controller::TrajectoryFollowerTranscription>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}



