#ifndef LOCOMOTION_MODE_H
#define LOCOMOTION_MODE_H

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
    LocomotionMode(rclcpp::NodeOptions options, std::string node_name);

  protected:
    // Node name which should be set by derived class.
    std::string node_name_;

    // Node can only work if it is enabled.
    bool enabled_;

    // Access parameters through the parameters_client_
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;

    // Transition names (loaded from config) to specify which robot_pose_transition is done when it is being dis-/enabled.
    // robot_pose_transition(TARGET_POSE)
    // TARGET_POSE = {CURRENT, POSE_1, POSE_2, etc.}

    // TODO: Does this need to be defined here?
    // Defines which positions are used in the enable and disable transition.
    struct RobotPose
    {
      std::vector<double> str_positions;
      std::vector<double> dep_positions;
    };
    std::map<std::string, std::shared_ptr<LocomotionMode::RobotPose>> poses_;

    std::shared_ptr<LocomotionMode::RobotPose> enable_pose_;
    std::shared_ptr<LocomotionMode::RobotPose> disable_pose_;

    std::string enable_pose_name_;
    std::string disable_pose_name_;

    std::vector<std::string> str_mapping_;
    std::vector<std::string> dep_mapping_;

    // Model
    std::shared_ptr<Rover> rover_;

    // Joints Pulisher
    rclcpp::Publisher<rover_msgs::msg::JointCommandArray>::SharedPtr joint_command_publisher_;
    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Load parameters
    void load_params();

    // Create rover model from urdf path
    void load_robot_model();

    // Initialize Messages
    // rover_msgs::msg::JointCommand joint_command_;
    rover_msgs::msg::JointCommandArray joint_command_array_;

    // Initialize Subscriber with callback function from derived class
    void enable_subscribers();

    // Disable Subscribers by changing their topic name and changing their callback function to a dummy class.
    void disable_subscribers();

    // Prototype functions that can be overwritten by the derived class.
    virtual bool enable();
    virtual bool disable();

    // Blocking transition to the input robot pose. Returns true once pose is sufficiently reached.
    // Robot pose consists of preprogrammed motor position and velocities.
    bool transition_to_robot_pose(std::string transition_name);

  private:
    // Services Objects
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;

    // Services Callbacks
    void enable_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disable_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;

    // Joint States Subscription
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Joint States Callback
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // URDF Model
    std::string model_path_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

  };

}
#endif
