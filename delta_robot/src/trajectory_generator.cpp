/// @file trajectory_generator.cpp
/// @brief Trajectory Generation Implementation for Delta Robot [Deprecated]
/// @author Sharwin Patil
/// @date 2025-07-13
/// @version 1.0
///
/// PARAMETERS:
///   None
///
/// PUBLISHERS:
///   None
///
/// SUBSCRIBERS:
///   None
///
/// SERVICES:
///   None
///
/// CLIENTS:
///   ~/delta_kinematics/delta_ik (deltarobot_interfaces::srv::DeltaIK): Client to compute inverse kinematics for trajectory points
///   ~/delta_kinematics/convert_to_joint_trajectory (deltarobot_interfaces::srv::ConvertToJointTrajectory): Client to convert end effector trajectory to joint trajectory
///   ~/delta_kinematics/convert_to_joint_vel_trajectory (deltarobot_interfaces::srv::ConvertToJointVelTrajectory): Client to convert end effector velocity trajectory to joint velocity trajectory

#include "rclcpp/rclcpp.hpp"
#include "trajectory_generator.hpp"

// Include all the custom messages and services
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>

template<typename T>
using ServiceResponseFuture = typename rclcpp::Client<T>::SharedFuture;

using Point = geometry_msgs::msg::Point;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;

DeltaTrajectoryGenerator::DeltaTrajectoryGenerator() : Node("delta_trajectory_generator") {
  RCLCPP_INFO(get_logger(), "DeltaTrajectoryGenerator node started");

  this->delta_ik_client = create_client<DeltaIK>("delta_kinematics/delta_ik");
  // Wait until service is ready
  while (!this->delta_ik_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->convert_to_joint_trajectory_client = create_client<ConvertToJointTrajectory>("delta_kinematics/convert_to_joint_trajectory");
  // Wait until service is ready
  while (!this->convert_to_joint_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->convert_to_joint_vel_trajectory_client = create_client<ConvertToJointVelTrajectory>("delta_kinematics/convert_to_joint_vel_trajectory");
  // Wait until service is ready
  while (!this->convert_to_joint_vel_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaTrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}