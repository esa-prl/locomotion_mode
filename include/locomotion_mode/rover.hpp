#ifndef ROVER_H
#define ROVER_H

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

    // Derive Position of Joint in static configuration
    static urdf::Pose get_parent_joint_position(const std::shared_ptr<urdf::Link> & link);

    // Trasposes position of child pose into the coordinate frame of the parent pose.
    static urdf::Pose transpose_pose(urdf::Pose parent, urdf::Pose child);

  private:
    // URDF Model
    std::shared_ptr<urdf::Model> model_;

    std::vector<std::shared_ptr<urdf::Joint>> joints_;
    std::vector<std::shared_ptr<urdf::Link>> links_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(const std::shared_ptr<urdf::Link> & start_link, std::string search_name);
  };

  struct Rover::Motor {
    Motor() {};
    Motor(std::shared_ptr<urdf::Link> init_link);

    std::shared_ptr<urdf::Joint> joint;
    std::shared_ptr<urdf::Link> link;
    urdf::Pose global_pose;

    struct State;

    std::shared_ptr<State> current_state;

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

    
    bool compute_wheel_diameter();
  };

}
#endif
