#include "locomotion_mode/stop_mode.hpp"

StopMode::StopMode(rclcpp::NodeOptions options, std::string node_name)
: LocomotionMode(options, node_name)
{
}

void StopMode::rover_velocities_callback(
  __attribute__((unused)) const geometry_msgs::msg::Twist::SharedPtr msg)
{

  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  rover_msgs::msg::JointCommand driving_msg;

  for (std::shared_ptr<LocomotionMode::Leg> leg : legs_) {

    driving_msg.name = leg->driving_motor->joint->name;
    driving_msg.mode = ("VELOCITY");
    driving_msg.value = 0;

    joint_command_array_msg.joint_command_array.push_back(driving_msg);

  }

  // Publish Message
  joint_command_publisher_->publish(joint_command_array_msg);
}


int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StopMode>(options, "stop_mode_node"));
  rclcpp::shutdown();
  return 0;
}
