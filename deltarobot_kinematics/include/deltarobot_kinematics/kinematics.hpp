#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_vel_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

using DeltaFK = deltarobot_interfaces::srv::DeltaFK;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using ConvertToJointVelTrajectory = deltarobot_interfaces::srv::ConvertToJointVelTrajectory;
using Point = geometry_msgs::msg::Point;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;

typedef struct {
  double x_vel;
  double y_vel;
  double z_vel;
} EEVelocity;

class DeltaKinematics : public rclcpp::Node {
public:
  DeltaKinematics();
  ~DeltaKinematics() = default;

private:
  rclcpp::Service<DeltaFK>::SharedPtr delta_fk_server;
  rclcpp::Service<DeltaIK>::SharedPtr delta_ik_server;
  rclcpp::Service<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_server;
  rclcpp::Service<ConvertToJointVelTrajectory>::SharedPtr convert_to_joint_vel_trajectory_server;
  
  // Service Callbacks
  void forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response);
  void inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response);
  void convertToJointTrajectory(const std::shared_ptr<ConvertToJointTrajectory::Request> request, std::shared_ptr<ConvertToJointTrajectory::Response> response);
  void convertToJointVelTrajectory(const std::shared_ptr<ConvertToJointVelTrajectory::Request> request, std::shared_ptr<ConvertToJointVelTrajectory::Response> response);

  // Helper function for IK to find active link angle when normal to the link's rotation axis (YZ-plane)
  int deltaIK_AngleYZ(float x0, float y0, float z0, float& theta);

  Point deltaFK(float theta1, float theta2, float theta3);
  DeltaJoints deltaIK(float x, float y, float z);

  // Jacobian Functions
  std::pair<std::vector<double>, std::vector<double>> calcAuxAngles(double theta1, double theta2, double theta3);
  Eigen::Matrix3d calcJacobian(double theta1, double theta2, double theta3);
  DeltaJointVels calcThetaDot(double theta1, double theta2, double theta3, double x_dot, double y_dot, double z_dot);
  // Helper functions for computing end-effector velocity
  std::vector<EEVelocity> computeGradient(const std::vector<Point>& position_data, double dt);
  
  /// @brief Base Triangle Side Length [mm]
  float SB;

  /// @brief End Effector Platform Triangle Side Length [mm]
  float SP;

  /// @brief Active Link Length [mm]
  float AL;

  /// @brief Passive Link Length [mm]
  float PL;

  /// @brief Passive Link Width [mm]
  float PW;

  /// @brief Joint Angle Min [rad]
  float JMin;

  /// @brief Joint Angle Max [rad]
  float JMax;

  /// @brief Phi Angles [rad]
  const float phi[3] = { -M_PI_2, M_PI / 6.0, (5.0 * M_PI) / 6.0 };
};

#endif  // KINEMATICS_HPP_