/// @file kinematics.cpp
/// @brief Kinematics Implementation for Delta Robot
///
/// PARAMETERS:
///   base_triangle_side_length (float64): The side length of the equilateral triangle defining the base [mm]
///   active_link_length (float64): The center distance (joint to joint) of the actuated links [mm]
///   passive_link_length (float64): The center distance (joint to joint) of the passive links [mm]
///   passive_link_width (float64): The width of the passive links [mm]
///   end_effector_side_length (float64): The side length of the equilateral triangle defining the end effector [mm]
///
/// SERVICES:
///   ~/delta_fk (deltarobot_interfaces::srv::DeltaFK): Computes the end effector position given the joint angles (forward kinematics)
///   ~/delta_ik (deltarobot_interfaces::srv::DeltaIK): Computes the joint angles given the end effector position (inverse kinematics)

#include "rclcpp/rclcpp.hpp"
#include "kinematics.hpp"
#include "rclcpp/node_options.hpp"

const float sqrt3 = sqrt(3.0);
constexpr float pi = 3.141592653;    // PI
const float sin120 = sqrt3 / 2.0;
constexpr float cos120 = -0.5;
const float tan60 = sqrt3;
constexpr float sin30 = 0.5;
const float tan30 = 1 / sqrt3;

DeltaKinematics::DeltaKinematics() : Node("delta_kinematics") {
  RCLCPP_INFO(this->get_logger(), "DeltaKinematics Started");

  // Save parameters from yaml for easy access
  this->SB = this->get_parameter("base_triangle_side_length").as_double();
  this->SP = this->get_parameter("end_effector_side_length").as_double();
  this->AL = this->get_parameter("active_link_length").as_double();
  this->PL = this->get_parameter("passive_link_length").as_double();
  this->PW = this->get_parameter("passive_link_width").as_double();

  // Update Kinematics Variables
  this->WB = (sqrt3 / 6) * this->SB;
  this->UB = (sqrt3 / 3) * this->SB;
  this->WP = (sqrt3 / 6) * this->SP;
  this->UP = (sqrt3 / 3) * this->SP;

  // Create FK and IK servers
  delta_fk_server = create_service<DeltaFK>(
    "delta_fk", 
    std::bind(&DeltaKinematics::forwardKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );
  delta_ik_server = create_service<DeltaIK>(
    "delta_ik",
    std::bind(&DeltaKinematics::inverseKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void DeltaKinematics::forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response) {
  // Locally save the request data (joint angles)
  float j1 = request->link1_angle;
  float j2 = request->link2_angle;
  float j3 = request->link3_angle;

  // Update the response data (end effector position)
  response->x = 0.0;
  response->y = 0.0;
  response->z = 0.0;
}

int DeltaKinematics::deltaFK_AngleYZ(float x0, float y0, float z0, float& theta) {
  float halfBase = 0.5 * this->SB;
  float y1 = -halfBase * tan30; // Half base * tan(30)
  y0 -= halfBase * this->SP;    // shift center to edge
  // z = a + b*y
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + this->AL * this->AL - this->PL * this->PL - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  // discriminant
  float d = -(a + b * y1) * (a + b * y1) + this->AL * (b * b * this->AL + this->AL);
  if (d < 0) return -1; // non-existing point
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
  float zj = a + b * yj;
  theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
  return 0;
}

void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {
  // Locally save the request data (end effector position)
  float x = request->x; // [mm]
  float y = request->y; // [mm]
  float z = request->z; // [mm]

  float theta1 = 0.0;
  float theta2 = 0.0;
  float theta3 = 0.0;

  int status = this->deltaFK_AngleYZ(x, y, z, theta1);
  if (status == 0) {
    status = this->deltaFK_AngleYZ(x * cos120 + y * sin120, y * cos120 - x * sin120, z, theta2);  // rotate coords to +120 deg
  } else {
    RCLCPP_ERROR(this->get_logger(), "DeltaIK: Non-existing point (%f, %f, %f)", x, y, z);
  }
  if (status == 0) {
    status = this->deltaFK_AngleYZ(x * cos120 - y * sin120, y * cos120 + x * sin120, z, theta3);  // rotate coords to -120 deg
  } else {
    RCLCPP_ERROR(this->get_logger(), "DeltaIK: Non-existing point (%f, %f, %f)", x, y, z);
  }

  // Update the response data (joint angles)
  response->link1_angle = theta1;
  response->link2_angle = theta2;
  response->link3_angle = theta3;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}