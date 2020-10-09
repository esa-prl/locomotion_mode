#ifndef ROVER_H
#define ROVER_H

#include <sensor_msgs/msg/joint_state.hpp>


namespace Rover{

  struct Motor
  {
    std::shared_ptr<urdf::Joint> joint;
    std::shared_ptr<urdf::Link> link;
    urdf::Pose global_pose;

    sensor_msgs::msg::JointState joint_state;
    Motor()
    {
      joint_state.name.resize(1);
      joint_state.position.resize(1);
      joint_state.velocity.resize(1);
      joint_state.effort.resize(1);
    }
  };

  struct Leg
  {
    // Leg name should be [LF (LeftFront), RM (RightMiddle), RR (RearRight), etc.)]
    std::string name;

    std::shared_ptr<Motor> driving_motor;
    std::shared_ptr<Motor> steering_motor;
    std::shared_ptr<Motor> deployment_motor;
    std::vector<std::shared_ptr<Motor>> motors;

    Leg()
    : driving_motor(std::make_shared<Motor>()),
      steering_motor(std::make_shared<Motor>()),
      deployment_motor(std::make_shared<Motor>())
    {
      motors.push_back(driving_motor);
      motors.push_back(steering_motor);
      motors.push_back(deployment_motor);
    }
  };

}
#endif