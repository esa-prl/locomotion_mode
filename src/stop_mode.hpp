#pragma once

#include "locomotion_mode/locomotion_mode.hpp"

namespace locomotion_mode {

class StopMode : public LocomotionMode
{
public:
  StopMode(rclcpp::NodeOptions options, std::string node_name);

private:
  rover_msgs::msg::JointCommandArray compute_joint_commands(const geometry_msgs::msg::Twist::SharedPtr msg);
};
}
