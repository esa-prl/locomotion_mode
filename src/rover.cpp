#include <locomotion_mode/rover.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace locomotion_mode;

Rover::Rover(std::string driving_name, std::string steering_name, std::string deployment_name, std::string model_path)
: model_(new urdf::Model())
{

  if (driving_name.empty() || steering_name.empty() || deployment_name.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("rover_parser"), "Identification strings is/are empty.");
    RCLCPP_ERROR(rclcpp::get_logger("rover_parser"),
      "Driving: [%s]\nSteering: [%s]\nDeployment: [%s]",
      driving_name.c_str(), steering_name.c_str(), deployment_name.c_str());
  }
  driving_name_ = driving_name;
  steering_name_ = steering_name;
  deployment_name_ = deployment_name;

  // Load Model from path
  if (!model_->initFile(model_path)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rover_parser"),
        "URDF file [%s] not found. Make sure the path is specified in the launch file.",
        model_path.c_str());
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("rover_parser"),
      "URDF file loaded successfully.");
  }
}

Rover::Motor::State::State() :
position(0),
velocity(0),
effort(0)
{}

Rover::Motor::Motor(std::shared_ptr<urdf::Link> init_link) :
current_state(std::make_shared<State>())
{
  // Init motor
  if (init_link->parent_joint->type == urdf::Joint::REVOLUTE ||
      init_link->parent_joint->type == urdf::Joint::CONTINUOUS ||
      init_link->parent_joint->type == urdf::Joint::PRISMATIC)
  {
    link = init_link;
    joint = link->parent_joint;
    global_pose = Rover::get_parent_joint_position(link);
  }
  else {
    RCLCPP_WARN(rclcpp::get_logger("rover_parser"), "Joint w/ name [%s] is of type [%u] which is not valid for a motor.", link->parent_joint->name.c_str());
  }
}

Rover::Leg::Leg()
: driving_motor(std::make_shared<Motor>()),
steering_motor(std::make_shared<Motor>()),
deployment_motor(std::make_shared<Motor>())
{
  motors.push_back(driving_motor);
  motors.push_back(steering_motor);
  motors.push_back(deployment_motor);
}

Rover::Leg::Leg(std::shared_ptr<Motor> drv_motor,
                std::shared_ptr<Motor> str_motor,
                std::shared_ptr<Motor> dep_motor)
: driving_motor(drv_motor),
steering_motor(str_motor),
deployment_motor(dep_motor)
{
  // Populate motors vector
  motors.push_back(driving_motor);
  motors.push_back(steering_motor);
  motors.push_back(deployment_motor);

  compute_wheel_diameter();

  // Find name for leg by keeping the last two digits of the joint name.
  name = driving_motor->joint->name;
  name.erase(name.begin(), name.end() - 2);
}

bool Rover::parse_model() {

  model_->getLinks(links_);

  // Loop through all links
  for (std::shared_ptr<urdf::Link> link : links_) {
    // Get Joints
    if (link->child_joints.size() != 0) {

      for (std::shared_ptr<urdf::Joint> child_joint : link->child_joints) {
        joints_.push_back(child_joint);
        // RCLCPP_INFO(rclcpp::get_logger("rover_parser"), "\t %s", child_joint->name.c_str());
      }
    }

    // Look for Driving link and create leg of locomotion model
    if (link->name.find(driving_name_) != std::string::npos) {    
      auto leg = std::make_shared<Leg>(std::make_shared<Rover::Motor>(link),
                                       std::make_shared<Rover::Motor>(get_link_in_leg(link, steering_name_)),
                                       std::make_shared<Rover::Motor>(get_link_in_leg(link, deployment_name_)));

      legs_.push_back(leg);
    }
  }

  return true;
}

// Find a link in the parents of the provided link which.
// The link is found if the search_name is in the link_name. They don't have to match fully.
std::shared_ptr<urdf::Link> Rover::get_link_in_leg(
  const std::shared_ptr<urdf::Link> & start_link, std::string search_name)
{

  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*start_link);

  while (tmp_link->parent_joint) {
    // If the search_name is found within the link name the search is aborted and said link is returned
    // TODO: Insert regex here
    if (tmp_link->parent_joint->name.find(search_name) != std::string::npos) {
      break;
    }
    tmp_link = tmp_link->getParent();
  }

  return tmp_link;
}

// Derive Position of Joint in static configuration
urdf::Pose Rover::get_parent_joint_position(const std::shared_ptr<urdf::Link> & link)
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
urdf::Pose Rover::transpose_pose(urdf::Pose parent, urdf::Pose child)
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

bool Rover::Leg::compute_wheel_diameter(){

  if (this->driving_motor->link->collision->geometry->type == urdf::Geometry::CYLINDER) {
    std::shared_ptr<urdf::Cylinder> cyl = std::static_pointer_cast<urdf::Cylinder>(
      this->driving_motor->link->collision->geometry);
    wheel_diameter = 2 * cyl->radius;

    return true;
  }
  else {
    wheel_diameter = 0.1;
    RCLCPP_WARN(
      rclcpp::get_logger("rover_parser"),
      "Wheel Link: %s collision geometry should be a cylinder! Wheel radius set to hardcoded value of %f [m]", this->driving_motor->link->name,
      wheel_diameter);

    return false;
  }
}
