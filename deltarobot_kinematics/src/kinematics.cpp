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

#include "kinematics.hpp"

DeltaKinematics::DeltaKinematics() : Node("delta_kinematics") {
  RCLCPP_INFO(this->get_logger(), "DeltaKinematics Started");

  // this->declare_parameter("qos_depth", 10);
  // int8_t qos_depth = 0;
  // this->get_parameter("qos_depth", qos_depth);

  // const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Declare parameters from yaml file
  this->declare_parameter("base_triangle_side_length", 180.0);
  this->declare_parameter("active_link_length", 120.0);
  this->declare_parameter("passive_link_length", 106.0);
  this->declare_parameter("passive_link_width", 20.0);
  this->declare_parameter("end_effector_side_length", 60.0);

  // Save parameters to struct for easy access
  this->robot_config.BL = this->get_parameter("base_triangle_side_length").as_double();
  this->robot_config.LL = this->get_parameter("active_link_length").as_double();
  this->robot_config.PL = this->get_parameter("passive_link_length").as_double();
  this->robot_config.PW = this->get_parameter("passive_link_width").as_double();
  this->robot_config.EEL = this->get_parameter("end_effector_side_length").as_double();

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
  double j1 = request->link1_angle;
  double j2 = request->link2_angle;
  double j3 = request->link3_angle;
  j1 = j1 * M_PI / 180.0;
  j2 = j2 * M_PI / 180.0;
  j3 = j3 * M_PI / 180.0;

  //TODO: Implement forward kinematics

  // Update the response data (end effector position)
  response->x = 0.0;
  response->y = 0.0;
  response->z = 0.0;
}
void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {
  // Locally save the request data (end effector position)
  double x = request->x;
  double y = request->y;
  double z = request->z;
  x = x * M_PI / 180.0;
  y = y * M_PI / 180.0;
  z = z * M_PI / 180.0;

  //TODO: Implement inverse kinematics

  // Update the response data (joint angles)
  response->link1_angle = 0.0;
  response->link2_angle = 0.0;
  response->link3_angle = 0.0;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}