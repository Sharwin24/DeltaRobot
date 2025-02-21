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
}

void DeltaTest::testTrajectory(
  [[maybe_unused]] std::shared_ptr<TestTraj::Request> request, std::shared_ptr<TestTraj::Response> response) {
  RCLCPP_INFO(get_logger(), "Received request for test trajectory");

  // Create a simple up and down trajectory ranging from (0, 0, -100) to (0, 0, -200)
  // Initial position is (0, 0, -100)
  // Final position is (0, 0, -200)
  // Use 10 points to interpolate the trajectory and use IK to obtain joint angles
  const int num_points = 10;
  std::vector<Point> end_effector_traj;

  Point initial_position;
  initial_position.x = 0.0;
  initial_position.y = 0.0;
  initial_position.z = -100.0;
  Point final_position;
  final_position.x = 0.0;
  final_position.y = 0.0;
  final_position.z = -200.0;
  
  // Add the initial position to the list
  end_effector_traj.push_back(initial_position);

  // Interpolate the rest of the points between the initial and final positions
  for (int i = 1; i < num_points - 1; i++) {
    Point intermediate_position;
    intermediate_position.x = 0.0;
    intermediate_position.y = 0.0;
    intermediate_position.z = initial_position.z - (i * 100.0 / (num_points - 1));
    end_effector_traj.push_back(intermediate_position);
  }
  end_effector_traj.push_back(final_position);

  // Log the created trajectory
  RCLCPP_INFO(get_logger(), "Trajectory created with %d points:", num_points);
  for (int i = 0; i < num_points; i++) {
    RCLCPP_INFO(get_logger(), "\tPoint %d: (%f, %f, %f)", i, end_effector_traj[i].x, end_effector_traj[i].y, end_effector_traj[i].z);
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