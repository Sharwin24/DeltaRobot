#ifndef DELTA_TRAJ_GEN_HPP_
#define DELTA_TRAJ_GEN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"

using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using Point = geometry_msgs::msg::Point;

class DeltaTrajectoryGenerator : public rclcpp::Node {
  public:
  DeltaTrajectoryGenerator();
  ~DeltaTrajectoryGenerator() = default;
  
  private:
  // Create a IK client to call the DeltaIK Service
  rclcpp::Client<DeltaIK>::SharedPtr delta_ik_client;
  rclcpp::Client<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_client;
};

#endif // !DELTA_TRAJ_GEN_HPP_