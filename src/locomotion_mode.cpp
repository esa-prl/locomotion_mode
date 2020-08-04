#include "locomotion_mode/locomotion_mode.hpp"

LocomotionMode::LocomotionMode(rclcpp::NodeOptions options, std::string node_name)
: Node(node_name,
    options.allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(true)),
  // Protected
  node_name_(node_name),
  enabled_(false),
  parameters_client_(std::make_shared<rclcpp::SyncParametersClient>(this)),
  enable_pose_(std::make_shared<RobotPose>()),
  disable_pose_(std::make_shared<RobotPose>()),
  model_(new urdf::Model()),
  // TODO: Add option to overwrite drive names. However, overwriting those should NOT be standard!
  // The names could be different for each robot or/and a locomotion mode.
  driving_name_("DRV"),
  steering_name_("STR"),
  deployment_name_("DEP"),
  joints_(),
  links_()
{
  // Load Parameters
  load_params();

  // Load URDF
  load_robot_model();

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

// Load Parameters
void LocomotionMode::load_params()
{
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Load urdf model path
  model_path_ = parameters_client_->get_parameters({"urdf_path"})[0].value_to_string();

  //// LOAD POSES
  std::string search_prefix = "poses";
  // Find all parameters which are prefixed by the search prefix
  std::vector<std::string> parameter_list = this->list_parameters({search_prefix}, 3).names;

  // Load joint mapping
  str_mapping_ = parameters_client_->get_parameters({"str_mapping"})[0].as_string_array();
  dep_mapping_ = parameters_client_->get_parameters({"dep_mapping"})[0].as_string_array();

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
            parameters_client_->get_parameters({parameter_name})[0].as_double_array();
        } else if (!positions_name.compare("str_positions")) {
          poses_[pose_name]->str_positions =
            parameters_client_->get_parameters({parameter_name})[0].as_double_array();
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
  enable_pose_name_ = parameters_client_->get_parameters({"enable_pose_name"})[0].value_to_string();
  disable_pose_name_ =
    parameters_client_->get_parameters({"disable_pose_name"})[0].value_to_string();

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
void LocomotionMode::load_robot_model()
{
  if (!model_->initFile(model_path_)) {
    RCLCPP_ERROR(
      this->get_logger(), "URDF file [%s] not found. Make sure the path is specified in the launch file.",
      model_path_.c_str());
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Successfully parsed urdf file.");
  }

  // Get Links
  model_->getLinks(links_);

  // Loop through all links
  for (std::shared_ptr<urdf::Link> link : links_) {
    // Get Joints
    if (link->child_joints.size() != 0) {
      for (std::shared_ptr<urdf::Joint> child_joint : link->child_joints) {
        joints_.push_back(child_joint);
        // RCLCPP_INFO(this->get_logger(), "\t %s", child_joint->name.c_str());
      }
    }

    // Look for Driving link and create leg of locomotion model
    if (link->name.find(driving_name_) != std::string::npos) {
      auto leg = std::make_shared<LocomotionMode::Leg>();
      init_motor(leg->driving_motor, link);

      // Find name for leg by keeping the last two digits of the joint name.
      std::string leg_name = leg->driving_motor->joint->name;
      leg_name.erase(leg_name.begin(), leg_name.end() - 2);
      leg->name = leg_name;


      RCLCPP_DEBUG(this->get_logger(), "LEG_NAME: [%s]", leg->name.c_str());

      if (leg->driving_motor->joint->type != urdf::Joint::REVOLUTE &&
        leg->driving_motor->joint->type != urdf::Joint::CONTINUOUS)
      {
        RCLCPP_WARN(
          this->get_logger(), "Driving Joint of Leg [%s] is of type [%u].",
          leg->name.c_str(), leg->driving_motor->joint->type);
      }

      legs_.push_back(leg);
    }
  }

  // Loop through all legs, find steering and deployment joints. Then save them into the leg.
  for (auto leg : legs_) {
    init_motor(
      leg->steering_motor,
      get_link_in_leg(leg->driving_motor->link, steering_name_));
    init_motor(
      leg->deployment_motor,
      get_link_in_leg(leg->driving_motor->link, deployment_name_));

    if (leg->steering_motor->joint->type != urdf::Joint::REVOLUTE &&
      leg->steering_motor->joint->type != urdf::Joint::CONTINUOUS)
    {
      RCLCPP_WARN(
        this->get_logger(), "Steering Joint of Leg [%s] is of type [%u].",
        leg->name.c_str(), leg->steering_motor->joint->type);
    }
    if (leg->deployment_motor->joint->type != urdf::Joint::REVOLUTE &&
      leg->deployment_motor->joint->type != urdf::Joint::CONTINUOUS)
    {
      RCLCPP_WARN(
        this->get_logger(), "Driving Joint of Leg [%s] is of type [%u].",
        leg->name.c_str(), leg->deployment_motor->joint->type);
    }
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
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "rover_motion_cmd_disabled", 10,
    std::bind(&LocomotionMode::rover_velocities_callback_disabled, this, std::placeholders::_1));
}

// Blocking function that returns true once a transition to a desired pose was achieved.
bool LocomotionMode::transition_to_robot_pose(std::string pose_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Transitioning to pose %s", pose_name.c_str());
  // Checks if pose_name is NONE and returns.
  if (!pose_name.compare("NONE")) {return true;} else {
    rover_msgs::msg::JointCommandArray joint_command_array_msg;
    rover_msgs::msg::JointCommand steering_msg;
    rover_msgs::msg::JointCommand deployment_msg;

    // Loops through legs
    for (std::shared_ptr<LocomotionMode::Leg> leg : legs_) {

      // Checks if leg is steerable
      if (leg->steering_motor->joint) {
        // Finds desired motor index (int) based on motor name.
        int index =
          std::distance(
          str_mapping_.begin(),
          std::find(str_mapping_.begin(), str_mapping_.end(), leg->name));

        steering_msg.name = leg->steering_motor->joint->name;
        steering_msg.mode = ("POSITION");
        steering_msg.value = poses_[pose_name]->str_positions[index];

        joint_command_array_msg.joint_command_array.push_back(steering_msg);

      }
      // Checks if leg is deployable
      if (leg->deployment_motor->joint) {
        // Finds desired motor index (int) based on motor name.
        int index =
          std::distance(
          dep_mapping_.begin(),
          std::find(dep_mapping_.begin(), dep_mapping_.end(), leg->name));

        deployment_msg.name = leg->deployment_motor->joint->name;
        deployment_msg.mode = ("POSITION");
        deployment_msg.value = poses_[pose_name]->dep_positions[index];
        joint_command_array_msg.joint_command_array.push_back(deployment_msg);
      }

    }

    joint_command_publisher_->publish(joint_command_array_msg);
    // TODO: Add wait to see if the position was actually reached.

    return true;
  }
}

// enable() and disable() are called from the enable and disable callback. They return true or false depending if the the mode was successfully dis-/enabled.
// enable() and disable() can be overwritten by the derived class to add aditional functionality on the enabling and disabling of the mode.
// Without overwrite they execute a transition to the en-/disable_pose and return if it was successful or not.
bool LocomotionMode::enable()
{
  return transition_to_robot_pose(enable_pose_name_);
}

bool LocomotionMode::disable()
{
  return transition_to_robot_pose(disable_pose_name_);
}


// Callback for the enable service
void LocomotionMode::enable_callback(
  __attribute__((unused)) const std_srvs::srv::Trigger::Request::SharedPtr request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Enabeling %s.", node_name_.c_str());
  if (enable()) {
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
  RCLCPP_DEBUG(this->get_logger(), "Disabling %s.", node_name_.c_str());

  // disable the subscribers before starting the disablign proceedure, so no rover_velocity callbacks can interfere with the transition
  disable_subscribers();

  if (disable()) {
    response->success = true;
    enabled_ = false;
  } else {
    response->success = false;
    RCLCPP_WARN(
      this->get_logger(), "Could not properly disable locomotion mode: %s", node_name_.c_str());
  }
}

// Dummy Callback in case someone actually sends a message to the disabled topic.
// TODO: There must be a better way to disable a subscription rather then just changing it's topic name to a new one.
void LocomotionMode::rover_velocities_callback_disabled(
  __attribute__((unused)) const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_WARN(
    this->get_logger(), "%s is disabled! Activate it before usage."
    "Why the f*** did you even send a message to this topic?!", node_name_.c_str());
}

// Dummy Callback function in case the derived class forgets to create a custom callback function
void LocomotionMode::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  RCLCPP_INFO(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_INFO(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_INFO(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_INFO(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_INFO(this->get_logger(), "Y_angular: %f.", msg->angular.y);

  RCLCPP_WARN(this->get_logger(), "Rover Velocities Callback was not overridden!");
}

// Define Link, joint and global position of a locomotion_mode motor.
void LocomotionMode::init_motor(
  std::shared_ptr<LocomotionMode::Motor> & motor,
  std::shared_ptr<urdf::Link> link)
{
  motor->link = link;
  motor->joint = link->parent_joint;
  motor->global_pose = get_parent_joint_position(link);
}

// Derive Position of Joint in static configuration
urdf::Pose LocomotionMode::get_parent_joint_position(std::shared_ptr<urdf::Link> & link)
{
  // TODO: Potentially pass by value instaed of shared_ptr so we don't have to copy it.
  // Copy link so we don't overwrite the original one
  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*link);

  urdf::Pose child_pose;

  while (tmp_link->parent_joint) {
    urdf::Pose parent_pose = tmp_link->parent_joint->parent_to_joint_origin_transform;

    child_pose = transpose_pose(parent_pose, child_pose);

    tmp_link = tmp_link->getParent();
  }
  return child_pose;
}

// Trasposes position of child pose into the coordinate frame of the parent pose.
urdf::Pose LocomotionMode::transpose_pose(urdf::Pose parent, urdf::Pose child)
{
  // Based on convention from Robot Dynamics of RSL@ETHZ. Also found on Hendriks RD-Summary
  urdf::Pose new_child;

  double & e0 = parent.rotation.w;
  double & e1 = parent.rotation.x;
  double & e2 = parent.rotation.y;
  double & e3 = parent.rotation.z;

  // Populate rotation matrix from parent quaternion.
  double c11 = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);
  double c12 = 2 * e1 * e2 - 2 * e0 * e3;
  double c13 = 2 * e0 * e2 + 2 * e1 * e3;
  double c21 = 2 * e0 * e3 + 2 * e1 * e2;
  double c22 = pow(e0, 2) - pow(e1, 2) + pow(e2, 2) - pow(e3, 2);
  double c23 = 2 * e2 * e3 - 2 * e0 * e1;
  double c31 = 2 * e1 * e3 - 2 * e0 * e2;
  double c32 = 2 * e0 * e1 + 2 * e2 * e3;
  double c33 = pow(e0, 2) - pow(e1, 2) - pow(e2, 2) + pow(e3, 2);

  // Populate parent translation vector.
  double & c14 = parent.position.x;
  double & c24 = parent.position.y;
  double & c34 = parent.position.z;

  // Compute transposed child by pos_child_in_parent_frame=_child_to_parent*pos_child_in_child_frame
  new_child.position.x = c11 * child.position.x + c12 * child.position.y + c13 * child.position.z +
    c14 * 1;
  new_child.position.y = c21 * child.position.x + c22 * child.position.y + c23 * child.position.z +
    c24 * 1;
  new_child.position.z = c31 * child.position.x + c32 * child.position.y + c33 * child.position.z +
    c34 * 1;

  // TODO Transform orientation

  return new_child;
}

// Find a link in the parents of the provided link which.
// The link is found if the search_name is in the link_name. They don't have to match fully.
std::shared_ptr<urdf::Link> LocomotionMode::get_link_in_leg(
  std::shared_ptr<urdf::Link> & start_link, std::string search_name)
{

  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*start_link);

  while (tmp_link->parent_joint) {
    // If the search_name is found within the link name the search is aborted and said link is returned
    if (tmp_link->parent_joint->name.find(search_name) != std::string::npos) {
      break;
    }
    tmp_link = tmp_link->getParent();
  }

  return tmp_link;
}


// Callback function, that saves the joint states into the class
// This is needed so the joint states can be used in the rover_velocity_callback_method of the derived class.
void LocomotionMode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (unsigned int i = 0; i < msg->name.size(); i++) {

    for (std::shared_ptr<LocomotionMode::Leg> leg : legs_) {
      for (std::shared_ptr<Motor> motor : leg->motors) {

        if (motor->joint->name.compare(msg->name[i].c_str()) == 0) {
          RCLCPP_DEBUG(this->get_logger(), "Received message for %s Motor.", msg->name[i].c_str());

          motor->joint_state.header = msg->header;
          if (!msg->position.empty()) {motor->joint_state.position[0] = msg->position[i];} else {
            RCLCPP_WARN(
              this->get_logger(), "Received no Position for Motor %s",
              msg->name[i].c_str());
          }

          if (!msg->velocity.empty()) {motor->joint_state.velocity[0] = msg->velocity[i];} else {
            RCLCPP_WARN(
              this->get_logger(), "Received no Veloctiy for Motor %s",
              msg->name[i].c_str());
          }

          if (!msg->effort.empty()) {motor->joint_state.effort[0] = msg->effort[i];}
          // else RCLCPP_WARN(this->get_logger(), "Received no Effort   for Motor %s", msg->name[i].c_str());
        }
      }
    }
  }
}
