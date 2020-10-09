#ifndef ROVER_H
#define ROVER_H

#include <sensor_msgs/msg/joint_state.hpp>

#include <urdf/model.h>


namespace RoverNS{

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

  class Rover
  {
  public:
    Rover(){};
    Rover(std::string driving_name, std::string steering_name, std::string deployment_name);
    Rover(std::shared_ptr<urdf::Model> model);

    std::vector<std::shared_ptr<RoverNS::Leg>> get_legs();
    
    bool parse_model(std::shared_ptr<urdf::Model> model);
    bool parse_model();

  private:

    // Raw URDF Model
    std::shared_ptr<urdf::Model> model_;

    std::vector<std::shared_ptr<urdf::Joint>> joints_;
    std::vector<std::shared_ptr<urdf::Link>> links_;


    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

    // Legs
    std::vector<std::shared_ptr<RoverNS::Leg>> legs_;

    bool init_motor(std::shared_ptr<RoverNS::Motor> & motor, std::shared_ptr<urdf::Link> link);

    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(std::shared_ptr<urdf::Link> & start_link, std::string search_name);

    urdf::Pose get_parent_joint_position(std::shared_ptr<urdf::Link> & link);

    urdf::Pose transpose_pose(urdf::Pose parent, urdf::Pose child);


  };

}
#endif