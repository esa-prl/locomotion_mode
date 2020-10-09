#ifndef STOP_MODE_H
#define STOP_MODE_H

#include "locomotion_mode/locomotion_mode.hpp"

namespace locomotion_mode {

  class StopMode : public LocomotionMode
  {
  public:
    StopMode(rclcpp::NodeOptions options, std::string node_name);

  private:
    void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  };
}

#endif
