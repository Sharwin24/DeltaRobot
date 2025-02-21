#include "rclcpp/rclcpp.hpp"
#include "delta_test.hpp"

// Include all the custom messages and services
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/msg/fk_trajectory.hpp"
#include "deltarobot_interfaces/msg/ik_trajectory.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/play_fk_trajectory.hpp"
#include "deltarobot_interfaces/srv/play_ik_trajectory.hpp"

DeltaTest::DeltaTest() : Node("delta_test") {
  RCLCPP_INFO(get_logger(), "DeltaTest node started");

  this->testing_trajectory_server = create_service<TestTraj>(
    "test_trajectory",
    std::bind(&DeltaTest::testTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void DeltaTest::testTrajectory(
  [[maybe_unused]] std::shared_ptr<TestTraj::Request> request, std::shared_ptr<TestTraj::Response> response) {
  RCLCPP_INFO(get_logger(), "Received request for test trajectory");

  //

  // Signal success after service is finished
  response->success = true;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaTest>());
  rclcpp::shutdown();
  return 0;
}