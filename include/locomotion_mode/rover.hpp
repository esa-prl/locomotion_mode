#ifndef ROVER_H
#define ROVER_H

#include <urdf/model.h>
#include <regex>

namespace locomotion_mode{

  class Rover
  {
  public:
    struct Leg;

    struct Motor;

    Rover(const std::string driving_name,
          const std::string steering_name,
          const std::string deployment_name,
          const std::string model_path);

    bool parse_model();
    
    // Legs
    std::vector<std::shared_ptr<Leg>> legs_;

  private:
    // Derive Position of Joint in static configuration
    static urdf::Pose get_parent_joint_position(const std::shared_ptr<urdf::Link> & link);

    // Trasposes position of child pose into the coordinate frame of the parent pose.
    static urdf::Pose transpose_pose(const urdf::Pose parent,
                                     const urdf::Pose child);
    
    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(const std::shared_ptr<urdf::Link> & start_link, const std::string search_name);
    
    // URDF Model
    std::shared_ptr<urdf::Model> model_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;
  };

  struct Rover::Motor {
    struct State;

    Motor() {};
    Motor(const std::shared_ptr<urdf::Link> init_link);

    std::shared_ptr<State> current_state;

    std::shared_ptr<urdf::Joint> joint;
    std::shared_ptr<urdf::Link> link;
    urdf::Pose global_pose;
  };

  struct Rover::Motor::State {
    double position;
    double velocity;
    double effort;
  };

  struct Rover::Leg {
    Leg();
    Leg(std::shared_ptr<Motor> drv_motor,
        std::shared_ptr<Motor> str_motor,
        std::shared_ptr<Motor> dep_motor);

    // Leg name should be [LF (LeftFront), RM (RightMiddle), RR (RearRight), etc.)]
    std::string name;

    std::shared_ptr<Motor> driving_motor;
    std::shared_ptr<Motor> steering_motor;
    std::shared_ptr<Motor> deployment_motor;
    std::vector<std::shared_ptr<Motor>> motors;
    
    double wheel_diameter;

    private:
      // Check if wheel link is cylindrical and use link diameter as wheel diameter
      bool compute_wheel_diameter();
  };

}
#endif
