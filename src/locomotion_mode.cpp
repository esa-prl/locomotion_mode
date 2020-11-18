#include "locomotion_mode/locomotion_mode.hpp"

#include <regex>

namespace locomotion_mode{

LocomotionMode::LocomotionMode(rclcpp::NodeOptions options, const std::string node_name)
: Node(node_name,
    options.allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(true)),
  // Protected
  node_name_(node_name),
  enabled_(false)
{
  // Load Parameters
  load_params();

  // Load URDF
  if(!load_robot_model()){
    RCLCPP_ERROR(this->get_logger(), "Loading of robot model failed!");
  }

  // Create Services
  enable_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/enable",
    std::bind(
      &LocomotionMode::enable_callback, this, std::placeholders::_1,
      std::placeholders::_2));
  disable_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/disable",
    std::bind(
      &LocomotionMode::disable_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Create Publishers
  joint_command_publisher_ = this->create_publisher<rover_msgs::msg::JointCommandArray>(
    "joint_cmds", 10);

  // Create Subscriptions
  joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(
      &LocomotionMode::joint_state_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "%s STARTED.", node_name.c_str());
}

// Dummy Callback function in case the derived class forgets to create a custom callback function
void LocomotionMode::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  joint_command_array_msg = compute_joint_commands(msg);

  // TODO: Check size of array
  // Publish Message
  joint_command_publisher_->publish(joint_command_array_msg);
}

rover_msgs::msg::JointCommandArray LocomotionMode::compute_joint_commands(
  __attribute__((unused)) const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "compute_joint_commands was not overridden!");
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  return joint_command_array_msg;
}

// Blocking function that returns true once a transition to a desired pose was achieved.
bool LocomotionMode::transition_to_robot_pose(const std::string pose_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Transitioning to pose %s", pose_name.c_str());
  // Checks if pose_name is NONE and returns.
  rover_msgs::msg::JointCommand joint_command_msg;
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  bool transition_to_pose = (pose_name != "NONE");

  for (auto leg : rover_->legs_) {
    // Stop all driving
    joint_command_msg.name = leg->driving_motor->joint->name;
    joint_command_msg.mode = ("VELOCITY");
    joint_command_msg.value = 0;

    joint_command_array_msg.joint_command_array.push_back(joint_command_msg);

    // Only send position commands if the pose shall be changed
    if (transition_to_pose) {
      // Checks if leg is steerable
      if (leg->steering_motor->joint) {
        // Finds desired motor index (int) based on motor name.
        int index =
          std::distance(
          str_mapping_.begin(),
          std::find(str_mapping_.begin(), str_mapping_.end(), leg->name));

        joint_command_msg.name = leg->steering_motor->joint->name;
        joint_command_msg.mode = ("POSITION");
        joint_command_msg.value = poses_[pose_name]->str_positions[index];

        joint_command_array_msg.joint_command_array.push_back(joint_command_msg);

      }
      // Checks if leg is deployable
      if (leg->deployment_motor->joint) {
        // Finds desired motor index (int) based on motor name.
        int index =
          std::distance(
          dep_mapping_.begin(),
          std::find(dep_mapping_.begin(), dep_mapping_.end(), leg->name));

        joint_command_msg.name = leg->deployment_motor->joint->name;
        joint_command_msg.mode = ("POSITION");
        joint_command_msg.value = poses_[pose_name]->dep_positions[index];
        joint_command_array_msg.joint_command_array.push_back(joint_command_msg);
      }

  }

    joint_command_publisher_->publish(joint_command_array_msg);
  }
  
  // TODO: Add wait to see if the position was actually reached.
  return true;
}

// enabling_sequence() and disabling_sequence() are called from the enable_ and disable_callback.
// They return true or false depending if the sequence was executed successfully.
// Both functions can be overwritten by the derived class to add aditional functionality on the enabling and disabling of the mode.
// Without overwrite they execute a transition to the en-/disable_pose and return if it was successful or not.
bool LocomotionMode::enabling_sequence()
{
  return transition_to_robot_pose(enable_pose_name_);
}

bool LocomotionMode::disabling_sequence()
{
  return transition_to_robot_pose(disable_pose_name_);
}

// Load Parameters
void LocomotionMode::load_params()
{
  // Load urdf model path
  model_path_ = this->get_parameter("urdf_path").as_string();

  driving_name_ = this->get_parameter("driving_identifier").as_string();
  steering_name_ = this->get_parameter("steering_identifier").as_string();
  deployment_name_ = this->get_parameter("deployment_identifier").as_string();

  leg_regex_string_ = this->get_parameter("leg_regex").as_string();

  //// LOAD POSES
  std::string search_prefix = "poses";
  // Find all parameters which are prefixed by the search prefix
  std::vector<std::string> parameter_list = this->list_parameters({search_prefix}, 3).names;

  // Load joint mapping
  str_mapping_ = this->get_parameters({"str_mapping"})[0].as_string_array();
  dep_mapping_ = this->get_parameters({"dep_mapping"})[0].as_string_array();

  // Create empty "NONE" pose.
  poses_["NONE"] = std::make_shared<LocomotionMode::RobotPose>();

  // Regular Expresion used to split the parameters from the parameter_list into tokens.
  std::regex regexp("\\.");

  // Loads joint positions of each pose
  for (auto parameter_name : parameter_list) {
    if (!parameter_name.compare("NONE")) {
      RCLCPP_WARN(this->get_logger(), "POSE NONE");
    } else {
      // Gets tokens of the parameters
      std::vector<std::string> tokens(
        std::sregex_token_iterator(parameter_name.begin(), parameter_name.end(), regexp, -1),
        std::sregex_token_iterator()
      );

      // Check if the initial token of the parameter name matches the search prefix.
      if (tokens[0].compare(search_prefix)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Search Prefix does not match parameter names. That should always match! Aborting.");
        return;
      }

      // Copy tokens for more intuitive coding bellow.
      std::string pose_name = tokens[1];
      std::string positions_name = tokens[2];

      // Check if the the pose already exists
      if (poses_.count(pose_name) == 0) {
        // Creates pose and saves it into the map.
        poses_[pose_name] = std::make_shared<LocomotionMode::RobotPose>();
      }

      // Try to save dep and str-positions into map.
      try {
        // Check where to save the positions of the given parameter
        if (!positions_name.compare("dep_positions")) {
          poses_[pose_name]->dep_positions =
            this->get_parameters({parameter_name})[0].as_double_array();
        } else if (!positions_name.compare("str_positions")) {
          poses_[pose_name]->str_positions =
            this->get_parameters({parameter_name})[0].as_double_array();
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "Positions name: [%s] does neither match str_positions nor dep_positions. Double check your config file.\n");
        }
      } catch (...) {
        RCLCPP_ERROR(
          this->get_logger(), "Did not find positions for pose with name: (%s)\n"
          "Make sure that the array consists of floats.", parameter_name.c_str());
      }
    }
  }

  // Print Poses for Validation
  for (auto pose : poses_) {
    RCLCPP_DEBUG(this->get_logger(), "\t---------------------");
    RCLCPP_DEBUG(this->get_logger(), "\tPose Name: (%s)", pose.first.c_str());

    RCLCPP_DEBUG(this->get_logger(), "\tSteering Positions:");
    for (unsigned i = 0; i < pose.second->str_positions.size(); ++i) {
      RCLCPP_DEBUG(
        this->get_logger(), "\t%s\t%f [RAD]",
        str_mapping_[i].c_str(), pose.second->str_positions[i]);
    }

    RCLCPP_DEBUG(this->get_logger(), "\tDeployment Positions:");
    for (unsigned i = 0; i < pose.second->dep_positions.size(); ++i) {
      RCLCPP_DEBUG(
        this->get_logger(), "\t%s\t%f [RAD]",
        dep_mapping_[i].c_str(), pose.second->dep_positions[i]);
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "\t---------------------");

  // Get names for enable and disable poses
  enable_pose_name_ = this->get_parameters({"enable_pose_name"})[0].value_to_string();
  disable_pose_name_ =
    this->get_parameters({"disable_pose_name"})[0].value_to_string();

  if (poses_.count(enable_pose_name_) == 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Enable position name [%s] was not found in poses! Enable position was set to \"NONE\"",
      enable_pose_name_.c_str());
    enable_pose_name_ = "NONE";
  }
  if (poses_.count(disable_pose_name_) == 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Disable position name [%s] was not found in poses! Disable position was set to \"NONE\"",
      disable_pose_name_.c_str());
    disable_pose_name_ = "NONE";
  }


}

// Load Robot Model (URDF or XACRO)
bool LocomotionMode::load_robot_model()
{
  rover_.reset(new Rover(driving_name_,
                         steering_name_,
                         deployment_name_,
                         model_path_,
                         leg_regex_string_));
  
  return rover_->parse_model();
}

// Callback for the enable service
void LocomotionMode::enable_callback(
  __attribute__((unused)) const std_srvs::srv::Trigger::Request::SharedPtr request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (enabled_) {
    RCLCPP_INFO(this->get_logger(), "Locomotion mode %s already enabled.", node_name_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Enabling locomotion mode %s.", node_name_.c_str());
  if (enabling_sequence()) {
    enable_subscribers();
    response->success = true;
    enabled_ = true;
  } else {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Could not enable locomotion mode: %s", node_name_.c_str());
  }
}

// Callback for the disable service
void LocomotionMode::disable_callback(
  __attribute__((unused)) const std_srvs::srv::Trigger::Request::SharedPtr request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!enabled_) {
    RCLCPP_INFO(this->get_logger(), "Locomotion mode %s already disabled.", node_name_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Disabling locomotion mode %s.", node_name_.c_str());

  // disable the subscribers before starting the disablign proceedure, so no rover_velocity callbacks can interfere with the transition
  disable_subscribers();

  if (disabling_sequence()) {
    response->success = true;
    enabled_ = false;
  } else {
    response->success = false;
    RCLCPP_WARN(
      this->get_logger(), "Could not properly disable locomotion mode: %s", node_name_.c_str());
  }
}

// Function to be called from the derived class while it is being initialized.
// Creates a subscriber using the (now by derived class overwritten) callback function
void LocomotionMode::enable_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "rover_motion_cmd", 10,
    std::bind(&LocomotionMode::rover_velocities_callback, this, std::placeholders::_1));
}

// Disable the subscribers
void LocomotionMode::disable_subscribers()
{
  rover_velocities_subscription_.reset();
}



// Callback function, that saves the joint states into the class
// This is needed so the joint states can be used in the rover_velocity_callback_method of the derived class.
void LocomotionMode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (unsigned int i = 0; i < msg->name.size(); i++) {
    for (auto leg : rover_->legs_) {
      for (auto motor : leg->motors) {
        // Check if motor is set. Can be unset in case no steering motor or deployment motor is present.
        if (motor->joint)
        {
          if (motor->joint->name.compare(msg->name[i].c_str()) == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Received message for %s Motor.", msg->name[i].c_str());

            if (!msg->position.empty()) {
              motor->current_state->position = msg->position[i];}
            else {
              RCLCPP_WARN(
                this->get_logger(), "Received no Position for Motor %s",
                msg->name[i].c_str());
            }

            if (!msg->velocity.empty()) {
              motor->current_state->velocity = msg->velocity[i];}
            else {
              RCLCPP_WARN(
                this->get_logger(), "Received no Veloctiy for Motor %s",
                msg->name[i].c_str());
            }

            if (!msg->effort.empty()) {
              motor->current_state->effort = msg->effort[i];}
            // else RCLCPP_WARN(this->get_logger(), "Received no Effort   for Motor %s", msg->name[i].c_str());
          }
        }
      }
    }
  }
}

}
