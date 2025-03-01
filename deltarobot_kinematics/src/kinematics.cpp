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
#include "rclcpp/node_options.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "kinematics.hpp"
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>

const float sqrt3 = sqrt(3.0);
const float sin120 = sqrt3 / 2.0;
constexpr float cos120 = -0.5;
const float tan60 = sqrt3;
constexpr float sin30 = 0.5;
const float tan30 = 1 / sqrt3;

using Point = geometry_msgs::msg::Point;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;

DeltaKinematics::DeltaKinematics() : Node("delta_kinematics") {
  RCLCPP_INFO(this->get_logger(), "DeltaKinematics Started");

  // Declare parameters
  this->declare_parameter("base_triangle_side_length", 200.0);
  this->declare_parameter("end_effector_side_length", 100.0);
  this->declare_parameter("active_link_length", 100.0);
  this->declare_parameter("passive_link_length", 100.0);
  this->declare_parameter("passive_link_width", 30.0);
  this->declare_parameter("joint_min", 0.0);
  this->declare_parameter("joint_max", M_PI / 2.0);

  // Save parameters from yaml for easy access
  this->SB = this->get_parameter("base_triangle_side_length").as_double();
  this->SP = this->get_parameter("end_effector_side_length").as_double();
  this->AL = this->get_parameter("active_link_length").as_double();
  this->PL = this->get_parameter("passive_link_length").as_double();
  this->PW = this->get_parameter("passive_link_width").as_double();
  this->JMin = this->get_parameter("joint_min").as_double();
  this->JMax = this->get_parameter("joint_max").as_double();

  // Create FK and IK servers
  delta_fk_server = create_service<DeltaFK>(
    "delta_fk", 
    std::bind(&DeltaKinematics::forwardKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );
  delta_ik_server = create_service<DeltaIK>(
    "delta_ik",
    std::bind(&DeltaKinematics::inverseKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create ConvertToJointTrajectory server
  convert_to_joint_trajectory_server = create_service<ConvertToJointTrajectory>(
    "convert_to_joint_trajectory",
    std::bind(&DeltaKinematics::convertToJointTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void DeltaKinematics::forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response) {
  std::vector<float> position = this->deltaFK(
    request->joint_angles.theta1, request->joint_angles.theta2, request->joint_angles.theta3
  );

  // Update the response data (end effector position)
  response->solution.x = position[0];
  response->solution.y = position[1];
  response->solution.z = position[2];
}

int DeltaKinematics::deltaIK_AngleYZ(float x0, float y0, float z0, float& theta) {
  float y1 = -0.5 * tan30 * this->SB; // Half base * tan(30)
  y0 -= 0.5 * tan30 * this->SP;    // shift center to edge
  // z = a + b*y
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + this->AL * this->AL - this->PL * this->PL - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  // discriminant
  float d = -(a + b * y1) * (a + b * y1) + this->AL * (b * b * this->AL + this->AL);
  if (d < 0) return -1; // non-existing point
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
  float zj = a + b * yj;
  theta = atan(-zj / (y1 - yj)) + ((yj > y1) ? M_PI : 0.0);
  return 0;
}

void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {
  std::vector<float> thetas = this->deltaIK(
    request->solution.x, request->solution.y, request->solution.z
  );

  // Update the response data (joint angles)
  response->joint_angles.theta1 = thetas[0];
  response->joint_angles.theta2 = thetas[1];
  response->joint_angles.theta3 = thetas[2];
}

void DeltaKinematics::convertToJointTrajectory(const std::shared_ptr<ConvertToJointTrajectory::Request> request, std::shared_ptr<ConvertToJointTrajectory::Response> response) {
  // Locally save the request data (end effector trajectory)
  std::vector<Point> trajectory = request->end_effector_trajectory;
  std::vector<DeltaJoints> joint_trajectory;

  // Iterate through the trajectory and convert each point to joint angles
  for (auto point : trajectory) {
    std::vector<float> thetas = this->deltaIK(point.x, point.y, point.z);

    DeltaJoints joint_angles;
    joint_angles.theta1 = thetas[0];
    joint_angles.theta2 = thetas[1];
    joint_angles.theta3 = thetas[2];

    joint_trajectory.push_back(joint_angles);
  }

  // Update the response data (joint trajectory)
  response->joint_trajectory = joint_trajectory;
}

std::vector<float> DeltaKinematics::deltaFK(float theta1, float theta2, float theta3) {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  float t = (this->SB - this->SP) * tan30 / 2;
  float y1 = -(t + this->AL * cos(theta1));
  float z1 = -this->AL * sin(theta1);
  float y2 = (t + this->AL * cos(theta2)) * sin30;
  float x2 = y2 * tan60;
  float z2 = -this->AL * sin(theta2);
  float y3 = (t + this->AL * cos(theta3)) * sin30;
  float x3 = -y3 * tan60;
  float z3 = -this->AL * sin(theta3);
  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - this->PL * this->PL);

  // discriminant
  float d = b * b - (float)4.0 * a * c;
  if (d < 0) {
    RCLCPP_ERROR(this->get_logger(), "DeltaFK: Invalid Configuration (%f, %f, %f) [rad]", theta1, theta2, theta3);
  } else {
    z = (-b + sqrt(d)) / (2 * a);
    x = (a1 * z + b1) / dnm;
    y = (a2 * z + b2) / dnm;
  }
  return std::vector<float>{x, y, z};
}

std::vector<float> DeltaKinematics::deltaIK(float x, float y, float z) {
  float theta1 = 0.0;
  float theta2 = 0.0;
  float theta3 = 0.0;

  int status = this->deltaIK_AngleYZ(x, y, z, theta1);
  if (status == 0) {
    status = this->deltaIK_AngleYZ(x * cos120 + y * sin120, y * cos120 - x * sin120, z, theta2);  // rotate coords to +120 deg
  } else {
    RCLCPP_ERROR(this->get_logger(), "DeltaIK: Non-existing point (%f, %f, %f) [mm]", x, y, z);
  }
  if (status == 0) {
    status = this->deltaIK_AngleYZ(x * cos120 - y * sin120, y * cos120 + x * sin120, z, theta3);  // rotate coords to -120 deg
  } else {
    RCLCPP_ERROR(this->get_logger(), "DeltaIK: Non-existing point (%f, %f, %f) [mm]", x, y, z);
  }
  return std::vector<float>{theta1, theta2, theta3};
}

std::pair<std::vector<double>, std::vector<double>> DeltaKinematics::calcAuxAngles(double theta1, double theta2, double theta3) {
  // First determine the end effector position using FK
  std::vector<float> position = this->deltaFK(theta1, theta2, theta3);

  const double UP = (sqrt3 / 3) * this->SP;
  std::vector<double> P = { position[0], position[1], position[2] };
  std::vector<double> D = { UP - this->AL, 0, 0 };
  
  std::vector<std::vector<double>> columns;
  for (int i = 0; i < 3; ++i) {
    std::vector<std::vector<double>> R = {
      {cos(this->phi[i]), sin(this->phi[i]), 0},
      {-sin(this->phi[i]), cos(this->phi[i]), 0},
      {0, 0, 1}
    };
    std::vector<double> c_i(3, 0.0);
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        c_i[j] += R[j][k] * P[k];
      }
      c_i[j] += D[j];
    }
    columns.push_back(c_i);
  }

  std::vector<std::vector<double>> C(3, std::vector<double>(3, 0.0)); // 3x3 matrix
  // C = [c_x1, c_x2, c_x3]
  //     [c_y1, c_y2, c_y3]
  //     [c_z1, c_z2, c_z3]
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i][j] = columns[j][i];
    }
  }
  double C_x2 = C[0][1];
  double C_y2 = C[1][1];
  double C_z2 = C[2][1];
  double C_x3 = C[0][2];
  double C_y3 = C[1][2];
  double C_z3 = C[2][2];
  // C_squared = c_xi^2 + c_yi^2 + c_zi^2
  double C_sqrd_2 = C_x2 * C_x2 + C_y2 * C_y2 + C_z2 * C_z2;
  double C_sqrd_3 = C_x3 * C_x3 + C_y3 * C_y3 + C_z3 * C_z3;
  // theta_3i = arccos(C_yi / PL)
  double t32 = acos(C_y2 / this->PL);
  double t33 = acos(C_y3 / this->PL);
  // k_numerator = c_xi ^ 2 + c_yi ^ 2 + c_zi ^ 2 - L ^ 2 - ELL ^ 2
  // k_denominator = 2 * L * ELL * sin(theta_3i)
  // theta_2i = arccos(k_numerator / k_denominator)
  double t22_numerator = C_sqrd_2 - this->AL * this->AL - this->PL * this->PL;
  double t22_denominator = 2 * this->AL * this->PL * sin(t32);
  double t23_numerator = C_sqrd_3 - this->AL * this->AL - this->PL * this->PL;
  double t23_denominator = 2 * this->AL * this->PL * sin(t33);
  double t22 = acos(t22_numerator / t22_denominator);
  double t23 = acos(t23_numerator / t23_denominator);
  // theta_1i is the actuated angles which were passed into the function
  // We only need to return the auxiliary angles
  // return std::vector<double>{t22, t23, t32, t33};
  return std::pair<std::vector<double>, std::vector<double>>{{theta2, t22, t23}, {theta3, t32, t33}};
}

Eigen::Matrix3d DeltaKinematics::calcJacobian(double theta1, double theta2, double theta3) {
  // The Jacobian matrix has 2 components: JTheta and Jp
  // Since this Jacobian will be used to compute the joint velocities, we need the inverse of JTheta
  // Jp * p_dot = JTheta * theta_dot -> theta_dot = JTheta_inv * Jp * p_dot
  
  // Obtain auxiliary angles
  auto aux_angles = this->calcAuxAngles(theta1, theta2, theta3);
  const std::vector<double> t1 = {theta1, theta2, theta3};
  const std::vector<double> t2 = aux_angles.first;
  const std::vector<double> t3 = aux_angles.second;
  double t22 = t2[1];
  double t23 = t2[2];
  double t32 = t3[1];
  double t33 = t3[2];

  // Jp Calculation
  auto J_ix = [this, &t1, &t2, &t3](int i) -> double {
    return sin(t3[i]) * cos(t2[i] + t1[i]) * cos(this->phi[i]) + cos(t3[i]) * sin(this->phi[i]);
  };
  auto J_iy = [this, &t1, &t2, &t3](int i) -> double {
    return -sin(t3[i]) * cos(t2[i] + t1[i]) * sin(this->phi[i]) + cos(t3[i]) * cos(this->phi[i]);
  };
  auto J_iz = [this, &t1, &t2, &t3](int i) -> double {
    return sin(t3[i]) * sin(t2[i] + t1[i]);
  };
  Eigen::Matrix3d Jp;
  for (int i = 0; i < 3; ++i) {
    Jp(i, 0) = J_ix(i);
    Jp(i, 1) = J_iy(i);
    Jp(i, 2) = J_iz(i);
  }

  // JTheta Calculation
  Eigen::Matrix3d JTheta;
  // Populate the diagonals with AL*sin(t2[i])*sin(t3[i])
  JTheta(0, 0) = this->AL * sin(theta2) * sin(theta3);
  JTheta(1, 1) = this->AL * sin(t22) * sin(t23);
  JTheta(2, 2) = this->AL * sin(t32) * sin(t33);

  // Invert JTheta
  Eigen::Matrix3d JTheta_inv = JTheta.inverse();
  return JTheta_inv * Jp;
}

std::vector<double> DeltaKinematics::calcThetaDot(double theta1, double theta2, double theta3, double x_dot, double y_dot, double z_dot) {
  Eigen::Matrix3d J = this->calcJacobian(theta1, theta2, theta3);
  Eigen::Vector3d p_dot(x_dot, y_dot, z_dot);
  Eigen::Vector3d theta_dot = J * p_dot;
  return std::vector<double>{theta_dot(0), theta_dot(1), theta_dot(2)};
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}