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
#include "geometry_msgs/msg/point.hpp"

using Point = geometry_msgs::msg::Point;

DeltaTest::DeltaTest() : Node("delta_test") {
  RCLCPP_INFO(get_logger(), "DeltaTest node started");

  this->testing_trajectory_server = create_service<TestTraj>(
    "test_trajectory",
    std::bind(&DeltaTest::testTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );

  this->delta_ik_client = create_client<deltarobot_interfaces::srv::DeltaIK>("delta_ik");
  // Wait until service is ready
  while (!this->delta_ik_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }
}

void DeltaTest::testTrajectory(
  [[maybe_unused]] std::shared_ptr<TestTraj::Request> request, std::shared_ptr<TestTraj::Response> response) {
  RCLCPP_INFO(get_logger(), "Received request for test trajectory");

  // Create a simple up and down trajectory ranging from (0, 0, -100) to (0, 0, -200)
  // Initial position is (0, 0, -100)
  // Final position is (0, 0, -200)
  // Use 10 points to interpolate the trajectory and use IK to obtain joint angles
  const int num_points = 10;
  std::vector<Point> trajectory;

  Point initial_position;
  initial_position.x = 0.0;
  initial_position.y = 0.0;
  initial_position.z = -100.0;
  Point final_position;
  final_position.x = 0.0;
  final_position.y = 0.0;
  final_position.z = -200.0;

  // Add the initial position to the list
  trajectory.push_back(initial_position);

  // Interpolate the rest of the points between the initial and final positions
  for (int i = 1; i < num_points - 1; i++) {
    Point intermediate_position;
    intermediate_position.x = 0.0;
    intermediate_position.y = 0.0;
    intermediate_position.z = initial_position.z - (i * 100.0 / (num_points - 1));
    trajectory.push_back(intermediate_position);
  }
  trajectory.push_back(final_position);

  // Log the created trajectory
  RCLCPP_INFO(get_logger(), "Trajectory created with %d points:", num_points);
  for (int i = 0; i < num_points; i++) {
    Point p = trajectory[i];
    RCLCPP_INFO(get_logger(), "\tPoint %d: (%.2f, %.2f, %.2f)", i, p.x, p.y, p.z);
  }

  // Convert the end-effector trajectory into a joint trajectory using the IK service
  std::vector<deltarobot_interfaces::msg::DeltaJoints> joint_trajectory;

  for (int i = 0; i < num_points; i++) {
    // Create IK request
    auto ik_request = std::make_shared<deltarobot_interfaces::srv::DeltaIK::Request>();
    ik_request->solution.x = trajectory[i].x;
    ik_request->solution.y = trajectory[i].y;
    ik_request->solution.z = trajectory[i].z;

    // Call the IK service and wait for the response before continuing
    auto result = this->delta_ik_client->async_send_request(ik_request);
    auto response = result.get(); // Block until service responds
  }

  // Log the joint trajectory
  RCLCPP_INFO(get_logger(), "Joint trajectory created with %d points:", num_points);
  for (int i = 0; i < num_points; i++) {
    deltarobot_interfaces::msg::DeltaJoints j = joint_trajectory[i];
    RCLCPP_INFO(get_logger(), "\tPoint %d: (%.2f, %.2f, %.2f)", i, j.theta1, j.theta2, j.theta3);
  }

  // Signal success after service is finished
  response->success = true;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaTest>());
  rclcpp::shutdown();
  return 0;
}