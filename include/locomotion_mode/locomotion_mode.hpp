#pragma once

#include <chrono>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "rover.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rover_msgs/msg/joint_command.hpp>
#include <rover_msgs/msg/joint_command_array.hpp>


using namespace std::chrono_literals;

namespace locomotion_mode {  

  class LocomotionMode : public rclcpp::Node
  {
  public:
    LocomotionMode(rclcpp::NodeOptions options, const std::string node_name);

  protected:
    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    virtual rover_msgs::msg::JointCommandArray compute_joint_commands(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Blocking transition to the input robot pose. Returns true once pose is sufficiently reached.
    // Robot pose consists of preprogrammed motor position and velocities.
    bool transition_to_robot_pose(const std::string pose_name);

    // Prototype functions that can be overwritten by the derived class.
    virtual bool enabling_sequence();
    virtual bool disabling_sequence();
    
    // Node name which should be set by derived class.
    std::string node_name_;

    // Names of poses which will be used after enabling and disabling a mode
    std::string enable_pose_name_;
    std::string disable_pose_name_;

    // Model
    std::shared_ptr<Rover> rover_;

    // Initialize Messages
    rover_msgs::msg::JointCommandArray joint_command_array_;

  private:
    // TODO: Does this need to be defined here?
    // Defines which positions are used in the enable and disable transition.
    struct RobotPose
    {
      std::vector<double> str_positions;
      std::vector<double> dep_positions;
    };
    
    // Load parameters
    void load_params();

    // Create rover model from urdf path
    bool load_robot_model();

    // Services Callbacks
    void enable_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disable_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Initialize Subscriber with callback function from derived class
    void enable_subscribers();

    // Disable Subscribers by changing their topic name and changing their callback function to a dummy class.
    void disable_subscribers();

    // Joint States Callback
    void joint_state_callback(
      const sensor_msgs::msg::JointState::SharedPtr msg);

    // Node can only work if it is enabled.
    bool enabled_;

    // Mapping of steering and deployment joints to poses array, both are specified in config.
    std::vector<std::string> str_mapping_;
    std::vector<std::string> dep_mapping_;
    
    std::map<std::string, std::shared_ptr<LocomotionMode::RobotPose>> poses_;

    // Services Objects
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;

    // Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;

    // Joint States Subscription
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Joints Pulisher
    rclcpp::Publisher<rover_msgs::msg::JointCommandArray>::SharedPtr joint_command_publisher_;

    // URDF Model
    std::string model_path_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

    std::string leg_regex_string_;
  };

}
