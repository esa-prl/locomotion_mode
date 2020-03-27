#ifndef LOCOMOTION_MODE_H
#define LOCOMOTION_MODE_H

#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>

#include <chrono>
#include <memory>

#include <string.h>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rover_msgs/msg/joint_command.hpp>
#include <rover_msgs/msg/joint_command_array.hpp>

#include <rover_msgs/srv/enable.hpp>
#include <rover_msgs/srv/disable.hpp>

using namespace std::chrono_literals;


class LocomotionMode : public rclcpp::Node
{
  public:
    LocomotionMode(rclcpp::NodeOptions options, std::string node_name);

    // TODO: Inside this namespace? 
    // TODO: What should be pointers, what not?
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
        std::shared_ptr<Motor> driving_motor;
        std::shared_ptr<Motor> steering_motor;
        std::shared_ptr<Motor> deployment_motor;
        std::vector<std::shared_ptr<Motor>> motors;

        Leg() :
        driving_motor(std::make_shared<Motor>()),
        steering_motor(std::make_shared<Motor>()),
        deployment_motor(std::make_shared<Motor>())
        {
            motors.push_back(driving_motor);
            motors.push_back(steering_motor);
            motors.push_back(deployment_motor);
        } 
    };

  protected:
    std::string node_name_;

    // Joints Pulisher
    rclcpp::Publisher<rover_msgs::msg::JointCommandArray>::SharedPtr joint_command_publisher_;

    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Initialize Subscriber with callback function from derived class
    void initialize_subscribers();

    // Load parameters
    void load_params();

    // Load robot model (urdf)
    void load_robot_model();

    // Initialize Messages
    // rover_msgs::msg::JointCommand joint_command_;
    rover_msgs::msg::JointCommandArray joint_command_array_;

    sensor_msgs::msg::JointState current_joint_state_;

    // Model
    std::shared_ptr<urdf::Model> model_;
    std::vector<std::shared_ptr<LocomotionMode::Leg>> legs_;

  private:

    // Services Objects
    rclcpp::Service<rover_msgs::srv::Enable>::SharedPtr  enable_service_;
    rclcpp::Service<rover_msgs::srv::Disable>::SharedPtr disable_service_;
 
    // Services Callbacks
    void enable(const rover_msgs::srv::Enable::Request::SharedPtr   request,
            std::shared_ptr<rover_msgs::srv::Enable::Response>      response);
    void disable(const rover_msgs::srv::Disable::Request::SharedPtr request,
            std::shared_ptr<rover_msgs::srv::Disable::Response>     response);

    // Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;    

    // Joint States Subscription
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Joint States Callback
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Model
    std::string model_name_;
    std::string model_dir_;
    std::string model_path_;

    std::string driving_name_;
    std::string steering_name_;
    std::string deployment_name_;

    std::vector<std::shared_ptr<urdf::Joint>> joints_;
    std::vector<std::shared_ptr<urdf::Link>> links_;

    void init_motor(std::shared_ptr<LocomotionMode::Motor> &motor, std::shared_ptr<urdf::Link> link);

    std::shared_ptr<LocomotionMode::Motor> init_motor(std::shared_ptr<urdf::Link> link);

    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(std::shared_ptr<urdf::Link> &start_link, std::string name);  

    urdf::Pose get_parent_joint_position(std::shared_ptr<urdf::Link> &link);

    urdf::Pose transpose_pose(urdf::Pose parent, urdf::Pose child);


    // Parameters
    // string mode_name_;

};

#endif