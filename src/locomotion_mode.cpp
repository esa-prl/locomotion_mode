#include "simple_rover_locomotion/locomotion_mode.hpp"

LocomotionMode::LocomotionMode(rclcpp::NodeOptions options)
: Node("locomotion_mode_node",
  options.allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true)),
  current_joint_state_(),
  model_(new urdf::Model()),
  // TODO: Readout from Config file
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
  activate_service_ = this->create_service<simple_rover_locomotion::srv::Activate>("activate", std::bind(&LocomotionMode::activate, this, std::placeholders::_1, std::placeholders::_2));
  changelocomotionmode_service_ = this->create_service<simple_rover_locomotion::srv::ChangeLocomotionMode>("change_locomotion_mode", std::bind(&LocomotionMode::change_locomotion_mode, this, std::placeholders::_1, std::placeholders::_2));

  // Create Publishers
  joint_command_publisher_ = this->create_publisher<rover_msgs::msg::JointCommandArray>("rover_joint_cmds", 10);
  
  // Create Subscriptions
  joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&LocomotionMode::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LocomotionMode initialized");
}

// Function to be called from the derived class while it is being initialized.
// Creates a subscriber using the (now by derived class overwritten) callback function
void LocomotionMode::initialize_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "rover_motion_cmd", 10, std::bind(&LocomotionMode::rover_velocities_callback, this, std::placeholders::_1));

}

void LocomotionMode::activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response)
{
    if (request->goal_state) {
        response->new_state = true;
    }
    else {response->new_state = false;}

    RCLCPP_INFO(this->get_logger(), "Goal State %d", request->goal_state);
    RCLCPP_INFO(this->get_logger(), "New State %d", response->new_state);
}

void LocomotionMode::change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response)
{
    // if (request->locomotion_mode == '2D_KINEMATICS' || request->locomotion_mode == 'WHEELWALKING') {
    //     response->response = 'LOCOMOTION CHANGED';
    // }
    // else {response->response = 'INVALID LOCOMOTION MODE REQUESTED';}

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", response->response.c_str());
    response->response = "SUCCSEFULLY SET " + request->locomotion_mode + " MODE!";
    RCLCPP_INFO(this->get_logger(), "Desired Locomotion Mode %s", request->locomotion_mode.c_str());
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

  // TODO: Tried delaring this a member variable, but didn't get it running. Check later if there are better tutorials for param loading. 2019-12-12
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
  
      // Get Driving links and create legs
      if (link->name.find(driving_name_) != std::string::npos) {
        auto leg = std::make_shared<LocomotionMode::Leg>();

        init_motor(leg->driving_motor, link);

        legs_.push_back(leg);
      }
    }

    // TODO: Check Joint type to be continuous or revolut?
    // Loop through all legs and find steering and deployment joints.
    for (std::shared_ptr<LocomotionMode::Leg> leg : legs_)
    {
      init_motor(leg->steering_motor, get_link_in_leg(leg->driving_motor->link, steering_name_));
      init_motor(leg->deployment_motor, get_link_in_leg(leg->driving_motor->link, deployment_name_));
    }


}

void LocomotionMode::init_motor(std::shared_ptr<LocomotionMode::Motor> &motor, std::shared_ptr<urdf::Link> link) {
  motor->link = link;
  
  motor->joint = link->parent_joint;

  motor->global_pose = get_parent_joint_position(link);
}

std::shared_ptr<LocomotionMode::Motor> LocomotionMode::init_motor(std::shared_ptr<urdf::Link> link) {
  // std::shared_ptr<LocomotionMode::Motor> motor;
  auto motor = std::make_shared<LocomotionMode::Motor>();

  motor->link = link;
  
  motor->joint = link->parent_joint;

  motor->global_pose = get_parent_joint_position(link);

  return motor;
}

// Derive Position of Joint in static configuration
// TODO: Pass value instaed of shared_ptr
urdf::Pose LocomotionMode::get_parent_joint_position(std::shared_ptr<urdf::Link> &link) {
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

urdf::Pose LocomotionMode::transpose_pose(urdf::Pose parent, urdf::Pose child)
{

  // Based on convention from Hendriks Summary
  // TODO Transform orientation
  urdf::Pose new_child;

  double &e0 = parent.rotation.w;
  double &e1 = parent.rotation.x;
  double &e2 = parent.rotation.y;
  double &e3 = parent.rotation.z;

  double c11 = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);
  double c12 = 2 * e1 * e2 - 2 * e0 * e3;
  double c13 = 2 * e0 * e2 + 2 * e1 * e3;
  double c21 = 2 * e0 * e3 + 2 * e1 * e2;
  double c22 = pow(e0, 2) - pow(e1, 2) + pow(e2, 2) - pow(e3, 2);
  double c23 = 2 * e2 * e3 - 2 * e0 * e1;
  double c31 = 2 * e1 * e3 - 2 * e0 * e2;
  double c32 = 2 * e0 * e1 + 2 * e2 * e3;
  double c33 = pow(e0, 2) - pow(e1, 2) - pow(e2, 2) + pow(e3, 2);

  double &c14 = parent.position.x;
  double &c24 = parent.position.y;
  double &c34 = parent.position.z;

  new_child.position.x = c11 * child.position.x + c12 * child.position.y + c13 * child.position.z + c14 * 1;
  new_child.position.y = c21 * child.position.x + c22 * child.position.y + c23 * child.position.z + c24 * 1;
  new_child.position.z = c31 * child.position.x + c32 * child.position.y + c33 * child.position.z + c34 * 1;

  return new_child;
}

std::shared_ptr<urdf::Link> LocomotionMode::get_link_in_leg(std::shared_ptr<urdf::Link> &start_link, std::string name) {

  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*start_link);

  while (tmp_link->parent_joint) {
    if (tmp_link->parent_joint->name.find(name) != std::string::npos) return tmp_link;
    tmp_link = tmp_link->getParent();
  }
  // Add return type that will cause if(return_object) to be false
}


// Save the joint states into the class
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