#ifndef ROVER_H
#define ROVER_H

#include <sensor_msgs/msg/joint_state.hpp>

#include <urdf/model.h>


namespace locomotion_mode{

  class Rover
  {
  public:
    Rover(std::string driving_name, std::string steering_name, std::string deployment_name, std::string model_path);

    struct Motor;

    struct Leg;

    // Legs
    std::vector<std::shared_ptr<Leg>> legs_;
    
    bool parse_model();

  private:

    // Raw URDF Model
    std::shared_ptr<urdf::Model> model_;

    std::vector<std::shared_ptr<urdf::Joint>> joints_;
    std::vector<std::shared_ptr<urdf::Link>> links_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

    bool init_motor(std::shared_ptr<Motor> & motor, std::shared_ptr<urdf::Link> link);

    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(std::shared_ptr<urdf::Link> & start_link, std::string search_name);

    // Derive Position of Joint in static configuration
    urdf::Pose get_parent_joint_position(const std::shared_ptr<urdf::Link> & link);

    // Trasposes position of child pose into the coordinate frame of the parent pose.
    urdf::Pose transpose_pose(urdf::Pose parent, urdf::Pose child);


  };

  struct Rover::Motor {
    Motor();

    std::shared_ptr<urdf::Joint> joint;
    std::shared_ptr<urdf::Link> link;
    urdf::Pose global_pose;

    sensor_msgs::msg::JointState joint_state;
  };

  struct Rover::Leg {
    Leg();
    // Leg name should be [LF (LeftFront), RM (RightMiddle), RR (RearRight), etc.)]
    std::string name;

    std::shared_ptr<Motor> driving_motor;
    std::shared_ptr<Motor> steering_motor;
    std::shared_ptr<Motor> deployment_motor;
    std::vector<std::shared_ptr<Motor>> motors;
    double wheel_diameter;

    
    bool compute_wheel_diameter();
  };

}
#endif