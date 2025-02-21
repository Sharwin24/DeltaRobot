#ifndef DELTA_TEST_HPP_
#define DELTA_TEST_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/srv/test_trajectory.hpp"

class DeltaTest : public rclcpp::Node {
public:
  DeltaTest();
  ~DeltaTest() = default;

private:
  using TestTraj = deltarobot_interfaces::srv::TestTrajectory;
  rclcpp::Service<TestTraj>::SharedPtr testing_trajectory_server;

  void testTrajectory(const std::shared_ptr<TestTraj::Request> request, std::shared_ptr<TestTraj::Response> response);
};

#endif // !DELTA_TEST_HPP_