#include <rclcpp/rclcpp.hpp>

#include "rosflight_msgs/msg/rc_raw.hpp"
#include "rosflight_msgs/msg/command.hpp"

namespace lqr_controller {

class RcTranscription : public rclcpp::Node {
public:
  // Constructor
  RcTranscription() : Node("rc_transcription") {
    rc_sub_ = this->create_subscription<rosflight_msgs::msg::RCRaw>(
      "rc_raw", 1, std::bind(&RcTranscription::rc_callback, this, std::placeholders::_1));
    command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 1);
  }


private:
  // ROS2 interfaces
  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_sub_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;

  void rc_callback(const rosflight_msgs::msg::RCRaw & msg) {
    rosflight_msgs::msg::Command out;

    out.header.stamp = this->get_clock()->now();
    out.mode = rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    out.fz = ((double) msg.values[2] - 1000) / 500;
    out.qx = ((double) msg.values[0] - 1500) / 500;
    out.qy = ((double) msg.values[1] - 1500) / 500;
    out.qz = ((double) msg.values[3] - 1500) / 500;

    command_pub_->publish(out);
  }
};

} // namespace lqr_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lqr_controller::RcTranscription>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}


