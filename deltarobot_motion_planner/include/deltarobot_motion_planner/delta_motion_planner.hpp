#ifndef DELTA_MOTION_PLANNER_HPP_
#define DELTA_MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/play_demo_trajectory.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"

using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaFK = deltarobot_interfaces::srv::DeltaFK;
using PlayDemoTraj = deltarobot_interfaces::srv::PlayDemoTrajectory;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using Point = geometry_msgs::msg::Point;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;

class DeltaMotionPlanner : public rclcpp::Node {
public:
  DeltaMotionPlanner();
  ~DeltaMotionPlanner() = default;

private:
  rclcpp::Publisher<DeltaJoints>::SharedPtr joint_pub;
  rclcpp::Service<PlayDemoTraj>::SharedPtr demo_traj_server;
  rclcpp::Client<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_client;
  rclcpp::Client<DeltaIK>::SharedPtr delta_ik_client;
  rclcpp::Client<DeltaFK>::SharedPtr delta_fk_client;

  void publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms = 50);

  void moveToPoint(const Point& point);
  void moveToConfiguration(const DeltaJoints& joints);

  void playDemoTrajectory(const std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response);
  
  std::vector<Point> straightUpDownTrajectory();
  std::vector<Point> pringleTrajectory();
};

#endif // !DELTA_MOTION_PLANNER_HPP_