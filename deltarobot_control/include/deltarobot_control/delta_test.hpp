#ifndef DELTA_TEST_HPP_
#define DELTA_TEST_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/test_trajectory.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"

class DeltaTest : public rclcpp::Node {
  public:
  DeltaTest();
  ~DeltaTest() = default;
  
  private:
  using TestTraj = deltarobot_interfaces::srv::TestTrajectory;
  rclcpp::Service<TestTraj>::SharedPtr testing_trajectory_server;
  rclcpp::Publisher<deltarobot_interfaces::msg::DeltaJoints>::SharedPtr joint_pub;
  
  // Create a IK client to call the DeltaIK Service
  rclcpp::Client<deltarobot_interfaces::srv::DeltaIK>::SharedPtr delta_ik_client;
  rclcpp::Client<deltarobot_interfaces::srv::ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_client;

  void testTrajectory(const std::shared_ptr<TestTraj::Request> request, std::shared_ptr<TestTraj::Response> response);

  using Point = geometry_msgs::msg::Point;
  std::vector<Point> straightUpDownTrajectory();
  std::vector<Point> pringleTrajectory();
};

#endif // !DELTA_TEST_HPP_