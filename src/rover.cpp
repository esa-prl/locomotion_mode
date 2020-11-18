#include <locomotion_mode/rover.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace locomotion_mode;

Rover::Rover(const std::string driving_name,
             const std::string steering_name,
             const std::string deployment_name,
             const std::string model_path,
             const std::string leg_regex_string)
: model_(new urdf::Model()),
leg_regex(leg_regex_string)
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

bool Rover::parse_model() {

  std::vector<std::shared_ptr<urdf::Link>> links;

  model_->getLinks(links);

  // Loop through all links
  for (std::shared_ptr<urdf::Link> link : links) {

    std::regex reg_exp("(?:^|_)"+driving_name_+"(?:$|_)");

    // Look for Driving link and create leg of locomotion model
    if (std::regex_search(link->name, reg_exp)) {
      // Derive name for leg by keeping the last two digits of the joint name.
      std::string leg_name;

      try {
        std::smatch match;
        // Search for leg name within the driving link name
        if (std::regex_search(link->name, match, leg_regex) && match.size() > 1) {
          // Set first captured group as leg name
          leg_name = match.str(1);
        }
        else {
          leg_name = std::string("LEG_NAME_NOT_FOUND");
          RCLCPP_ERROR(rclcpp::get_logger("rover_parser"),
            "No leg name matched.");
        }
      } catch (std::regex_error &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rover_parser"),
          "Syntax error in the leg name regular expression.");
      }

      // Create leg, based on name, driving, steering and deployment links
      auto leg = std::make_shared<Leg>(leg_name,
                                       std::make_shared<Rover::Motor>(link),
                                       std::make_shared<Rover::Motor>(get_link_in_leg(link, steering_name_)),
                                       std::make_shared<Rover::Motor>(get_link_in_leg(link, deployment_name_)));

      legs_.push_back(leg);
    }
  }

  return true;
}

// Derive Position of Joint in static configuration
urdf::Pose Rover::get_parent_joint_position(const std::shared_ptr<urdf::Link> & link)
{
  // Copy link so we don't overwrite the original one
  std::shared_ptr<urdf::Link> tmp_link = link;

  urdf::Pose child_pose;

  while (tmp_link->parent_joint) {
    urdf::Pose parent_pose = tmp_link->parent_joint->parent_to_joint_origin_transform;

    child_pose = transpose_pose(parent_pose, child_pose);

    tmp_link = tmp_link->getParent();
  }
  return child_pose;
}

// Trasposes position of child pose into the coordinate frame of the parent pose.
urdf::Pose Rover::transpose_pose(const urdf::Pose parent, const urdf::Pose child)
{
  // Based on convention from Robot Dynamics of RSL@ETHZ. Also found on Hendriks RD-Summary
  urdf::Pose new_child;

  double e0 = parent.rotation.w;
  double e1 = parent.rotation.x;
  double e2 = parent.rotation.y;
  double e3 = parent.rotation.z;

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
  double c14 = parent.position.x;
  double c24 = parent.position.y;
  double c34 = parent.position.z;

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
std::shared_ptr<urdf::Link> Rover::get_link_in_leg(
  const std::shared_ptr<urdf::Link> & start_link, const std::string search_name)
{
  std::regex reg_exp("(?:^|_)"+search_name+"(?:$|_)");

  // Copy link so we don't overwrite the original one
  std::shared_ptr<urdf::Link> tmp_link = start_link;

  while (tmp_link->parent_joint) {
    // If the search_name is found within the link name the search is aborted and said link is returned
    if(std::regex_search(tmp_link->parent_joint->name, reg_exp)) {
      return tmp_link;
    }
    tmp_link = tmp_link->getParent();
  }

  return std::make_shared<urdf::Link>();
}

Rover::Motor::Motor(const std::shared_ptr<urdf::Link> init_link) :
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

Rover::Leg::Leg(std::string leg_name,
                std::shared_ptr<Motor> drv_motor,
                std::shared_ptr<Motor> str_motor,
                std::shared_ptr<Motor> dep_motor)
: name(leg_name),
driving_motor(drv_motor),
steering_motor(str_motor),
deployment_motor(dep_motor)
{
  // Populate motors vector
  motors.push_back(driving_motor);
  motors.push_back(steering_motor);
  motors.push_back(deployment_motor);

  compute_wheel_diameter();
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
