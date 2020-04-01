#include "locomotion_mode/locomotion_mode.hpp"

LocomotionMode::LocomotionMode(rclcpp::NodeOptions options, std::string node_name)
: Node(node_name,
  options.allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true)),
  node_name_(node_name),
  current_joint_state_(),
  model_(new urdf::Model()),
  // TODO: Readout transitions names from config file
  enable_pose_name_("NONE"),
  disable_pose_name_("NONE"),
  enabled_(false),
  // TODO: Readout transitions joint values from config file
  // TODO: Readout Names from Config file
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
    "~/enable", std::bind(&LocomotionMode::enable_callback, this, std::placeholders::_1, std::placeholders::_2));
  disable_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/disable", std::bind(&LocomotionMode::disable_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Create Publishers
  joint_command_publisher_ = this->create_publisher<rover_msgs::msg::JointCommandArray>("joint_cmds", 10);
  
  // Create Subscriptions
  joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&LocomotionMode::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LocomotionMode initialized");
}

// Function to be called from the derived class while it is being initialized.
// Creates a subscriber using the (now by derived class overwritten) callback function
void LocomotionMode::enable_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "rover_motion_cmd", 10, std::bind(&LocomotionMode::rover_velocities_callback, this, std::placeholders::_1));
}

// Disable the subscribers
void LocomotionMode::disable_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "rover_motion_cmd_disabled", 10, std::bind(&LocomotionMode::rover_velocities_callback_disabled, this, std::placeholders::_1));
}

// Blocking function that returns true once a transition to a desired pose was achieved.
bool LocomotionMode::transition_to_robot_pose(std::string transition_name) {
  RCLCPP_INFO(this->get_logger(), "Transitioning to pose %s", transition_name.c_str());
  // TODO: Implement real transition and pose data format. Would make sense to have it loaded in the urdf/xacro robot model.
  // string compare outputs 0 if the strings are identical
  if (!transition_name.compare("NONE")) return true;
  else {
    // Check here if the name has a motor values corresponding to the pose name.
    return false;
  }
}

// enable and disable are called from the enable and disable callback. They return true or false depending if the the mode was successfully dis-/enabled.
// Enable and Disable can be overwritten by the derived class to add aditional functionality on the enabling and disabling of the mode.
// Without overwrite they execute a transition to the en-/disable_pose and return if it was successful or not.
bool LocomotionMode::enable() {
  return transition_to_robot_pose(enable_pose_name_);
}

bool LocomotionMode::disable() {
  return transition_to_robot_pose(disable_pose_name_);
}


// Callback for the enable service
void LocomotionMode::enable_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
  RCLCPP_INFO(this->get_logger(), "Someone requested to enable %s.", node_name_.c_str());    
  if (enable())
  {
    enable_subscribers();
    response->success = true;
    enabled_ = true;
  }
  else {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Could not enable locomotion mode: %s", node_name_.c_str());    
  }
}

// Callback for the disable service
void LocomotionMode::disable_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
  RCLCPP_INFO(this->get_logger(), "Someone requested to disable this locomotion mode.");    
  
  // disable the subscribers before starting the disablign proceedure, so no rover_velocity callbacks can interfere with the transition
  disable_subscribers();
  
  if (disable()){
    response->success = true;
    enabled_ = false;
  }
  else {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Could not properly disable locomotion mode: %s", node_name_.c_str());
  }

}

// Dummy Callback in case someone actually sends a message to the disabled topic.
// TODO: There must be a better way to disable a subscription rather then just changing it's topic name to a new one.
void LocomotionMode::rover_velocities_callback_disabled(const geometry_msgs::msg::Twist::SharedPtr msg) {
  RCLCPP_WARN(this->get_logger(), "%s is disabled! Activate it before usage. Why the f*** did you even send a message to this topic?!", node_name_.c_str());
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

// Load Parameters
void LocomotionMode::load_params()
{
  // Look in /demos/demo_nodes_cpp/src/parameters/set_and_get_parameters.cpp for implementation examples
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  model_path_ = parameters_client->get_parameters({"urdf_model_path"})[0].value_to_string();
}

// Load Robot Model (URDF or XACRO)
void LocomotionMode::load_robot_model()
{
    if (!model_->initFile(model_path_)){
      RCLCPP_ERROR(this->get_logger(), "URDF file [%s] not found. Make sure the path is specified in the launch file.", model_path_.c_str());
    }
    else RCLCPP_INFO(this->get_logger(), "Successfully parsed urdf file.");
 
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
        legs_.push_back(leg);
      }
    }

    // TODO: Check Joint type to be continuous or revolut?
    // Loop through all legs, find steering and deployment joints. Then save them into the leg.
    for (std::shared_ptr<LocomotionMode::Leg> leg : legs_)
    {
      init_motor(leg->steering_motor, get_link_in_leg(leg->driving_motor->link, steering_name_));
      init_motor(leg->deployment_motor, get_link_in_leg(leg->driving_motor->link, deployment_name_));
    }
}

// Define Link, joint and global position of a locomotion_mode motor.
void LocomotionMode::init_motor(std::shared_ptr<LocomotionMode::Motor> &motor, std::shared_ptr<urdf::Link> link) {
  motor->link = link;
  motor->joint = link->parent_joint;
  motor->global_pose = get_parent_joint_position(link);
}

// Derive Position of Joint in static configuration
urdf::Pose LocomotionMode::get_parent_joint_position(std::shared_ptr<urdf::Link> &link) {
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

  double &e0 = parent.rotation.w;
  double &e1 = parent.rotation.x;
  double &e2 = parent.rotation.y;
  double &e3 = parent.rotation.z;

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
  double &c14 = parent.position.x;
  double &c24 = parent.position.y;
  double &c34 = parent.position.z;

  // Compute transposed child by pos_child_in_parent_frame=_child_to_parent*pos_child_in_child_frame
  new_child.position.x = c11 * child.position.x + c12 * child.position.y + c13 * child.position.z + c14 * 1;
  new_child.position.y = c21 * child.position.x + c22 * child.position.y + c23 * child.position.z + c24 * 1;
  new_child.position.z = c31 * child.position.x + c32 * child.position.y + c33 * child.position.z + c34 * 1;

  // TODO Transform orientation

  return new_child;
}

// Find a link in the parents of the provided link which. The link is found if the search_name is in the link_name. They don't have to match fully. 
std::shared_ptr<urdf::Link> LocomotionMode::get_link_in_leg(std::shared_ptr<urdf::Link> &start_link, std::string name) {

  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*start_link);

  while (tmp_link->parent_joint) {
    if (tmp_link->parent_joint->name.find(name) != std::string::npos) break;
    tmp_link = tmp_link->getParent();
  }

  return tmp_link;
}


// Callback function, that saves the joint states into the class
// This is needed so the joint states can be used in the rover_velocity_callback_method of the derived class.
void LocomotionMode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (unsigned int i = 0; i < msg->name.size(); i++) {
    
    for (std::shared_ptr<LocomotionMode::Leg> leg : legs_)
    {
      for (std::shared_ptr<Motor> motor : leg->motors){

        if (motor->joint->name.compare(msg->name[i].c_str()) == 0) {
          RCLCPP_DEBUG(this->get_logger(), "Received message for %s Motor.", msg->name[i].c_str());
          
          motor->joint_state.header = msg->header;
          if (!msg->position.empty()) motor->joint_state.position[0] = msg->position[i];
          else RCLCPP_WARN(this->get_logger(), "Received no Position for Motor %s", msg->name[i].c_str());

          if (!msg->velocity.empty()) motor->joint_state.velocity[0] = msg->velocity[i];
          else RCLCPP_WARN(this->get_logger(), "Received no Veloctiy for Motor %s", msg->name[i].c_str());
          
          if (!msg->effort.empty())   motor->joint_state.effort[0]   = msg->effort[i];
          else RCLCPP_WARN(this->get_logger(), "Received no Effort   for Motor %s", msg->name[i].c_str());
        }
      } 
    }
  }
}