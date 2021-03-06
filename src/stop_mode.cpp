#include "stop_mode.hpp"

namespace locomotion_mode{

StopMode::StopMode(rclcpp::NodeOptions options, std::string node_name)
: LocomotionMode(options, node_name)
{
}

rover_msgs::msg::JointCommandArray StopMode::compute_joint_commands(
  __attribute__((unused)) const geometry_msgs::msg::Twist::SharedPtr msg)
{

  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  rover_msgs::msg::JointCommand driving_msg;

  // Set driving motor velocity to 0
  for (auto leg : rover_->legs_) {
    driving_msg.name = leg->driving_motor->joint->name;
    driving_msg.mode = ("VELOCITY");
    driving_msg.value = 0;

    joint_command_array_msg.joint_command_array.push_back(driving_msg);
  }

  return joint_command_array_msg;
}

}

int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<locomotion_mode::StopMode>(options, "stop_mode_node"));
  rclcpp::shutdown();
  return 0;
}
