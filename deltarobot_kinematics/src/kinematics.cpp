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
void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {
  // Locally save the request data (end effector position)
  float x = request->x; // [mm]
  float y = request->y; // [mm]
  float z = request->z; // [mm]

  // Tangent Half Angle Substitution
  const float A = this->WB - this->UP;
  const float B = (this->SP / 2) - ((sqrt3 / 2) * this->WP);
  const float C = this->WP - (this->WB / 2);

  /// Tangent Half Angle Coefficients
  const float E[3] = {
    2 * this->AL*(y+A),
    -AL * (sqrt3 * (x+B) + y + C),
    AL * (sqrt3 * (x-B) - y - C)
  };

  const float F = 2 * z * this->AL;

  const float r2 = x*x + y*y + z*z;
  const float AL2 = this->AL * this->AL;
  const float PL2 = this->PL * this->PL;
  const float G[3] = {
    r2 + A + AL2 + 2*y*A - PL2,
    r2 + B + AL2 + 2*(x*B+y*C) - PL2,
    r2 + B + AL2 + 2*(-x*B-y*C) - PL2
  };

  float thetas[3] = {0.0, 0.0, 0.0};
  for (uint8_t i = 0; i < 3; i++) {
    float D = E[i]*E[i] + F*F - G[i]*G[i];
    if (D < 0) {
      // Non-existing point
      RCLCPP_ERROR(this->get_logger(), "Delta IK: Non-existing point -> (x: %f, y: %f, z: %f)", x, y, z);
      thetas[0] = 0.0;
      thetas[1] = 0.0;
      thetas[2] = 0.0;
      thetas[i] = -1.0; // Error flag
      break;
    }
    float sqrtD = sqrt(D);
    float theta_plus = (-F + sqrtD) / (G[i] - E[i]);
    float theta_minus = (-F - sqrtD) / (G[i] - E[i]);
    theta_plus = 2 * atan(theta_plus);
    theta_minus = 2 * atan(theta_minus);
    // Pick the solution with the knees "kinked out", the angle should be closer to zero
    if (abs(theta_plus) < abs(theta_minus)) {
      thetas[i] = theta_plus;
    } else {
      thetas[i] = theta_minus;
    }
  }

  // Update the response data (joint angles)
  response->link1_angle = thetas[0];
  response->link2_angle = thetas[1];
  response->link3_angle = thetas[2];
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}