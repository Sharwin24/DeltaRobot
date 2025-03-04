#include "rclcpp/rclcpp.hpp"
#include "delta_motion_planner.hpp"

// Include all the custom messages and services
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/msg/fk_trajectory.hpp"
#include "deltarobot_interfaces/msg/ik_trajectory.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/play_fk_trajectory.hpp"
#include "deltarobot_interfaces/srv/play_ik_trajectory.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>

template<typename T>
using ServiceResponseFuture = typename rclcpp::Client<T>::SharedFuture;

using Point = geometry_msgs::msg::Point;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using PlayDemoTraj = deltarobot_interfaces::srv::PlayDemoTrajectory;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;

DeltaMotionPlanner::DeltaMotionPlanner() : Node("delta_motion_planner") {
  RCLCPP_INFO(get_logger(), "DeltaMotionPlanner node started");

  this->demo_traj_server = create_service<PlayDemoTraj>(
    "play_demo_trajectory",
    std::bind(&DeltaMotionPlanner::playDemoTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );

  this->joint_pub = this->create_publisher<DeltaJoints>("set_joints", 10);

  this->delta_ik_client = create_client<DeltaIK>("delta_ik");
  // Wait until service is ready
  while (!this->delta_ik_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->delta_fk_client = create_client<DeltaFK>("delta_fk");
  // Wait until service is ready
  while (!this->delta_fk_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->convert_to_joint_trajectory_client = create_client<ConvertToJointTrajectory>("convert_to_joint_trajectory");
  // Wait until service is ready
  while (!this->convert_to_joint_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }
}

void DeltaMotionPlanner::publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms) {
  // Publish the joint trajectory to the motors with a small delay [ms] between each point
  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    this->joint_pub->publish(joint_traj[i]);
    rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

void DeltaMotionPlanner::moveToPoint(const Point& point) {
  // Perform IK to get the joint angles and to ensure if the point is reachable
  auto ik_request = std::make_shared<DeltaIK::Request>();
  ik_request->solution = point;

  auto future_result = this->delta_ik_client->async_send_request(
    ik_request,
    [this](ServiceResponseFuture<DeltaIK> future) {
      auto response = future.get();
      if (response->success) {
        // If the IK solution is valid, move to the point
        std::vector<DeltaJoints> joint_traj = {response->joint_angles};
        this->publishMotorCommands(joint_traj, 0);
      } else {
        RCLCPP_ERROR(get_logger(), "IK solution not found for the given end effector point");
      }
    }
  );
}

void DeltaMotionPlanner::moveToConfiguration(const DeltaJoints& joints) {
  // Before publishing joint angles, ensure the request is valid using FK
  auto fk_request = std::make_shared<DeltaFK::Request>();
  fk_request->joint_angles = joints;

  auto future_result = this->delta_fk_client->async_send_request(
    fk_request,
    [this, joints](ServiceResponseFuture<DeltaFK> future) {
      auto response = future.get();
      if (response->success) {
        // If the FK solution is valid, move to the configuration
        std::vector<DeltaJoints> joint_traj = {joints};
        this->publishMotorCommands(joint_traj, 0);
      } else {
        RCLCPP_ERROR(get_logger(), "FK solution not found for the given joint angles");
      }
    }
  );
}

void DeltaMotionPlanner::playDemoTrajectory(
  std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response) {

  std::string type = request->type.data;
  std::vector<Point> trajectory;
  if (type == "up_down") {
    trajectory = this->straightUpDownTrajectory();
  } else if (type == "pringle") {
    trajectory = this->pringleTrajectory();
  } else {
    const std::vector<std::string> available_demos = {"up_down", "pringle"};
    RCLCPP_ERROR(get_logger(), "Invalid demo trajectory: %s", type.c_str());
    RCLCPP_ERROR(get_logger(), "Available demo trajectories: %s", std::accumulate(
      std::next(available_demos.begin()), available_demos.end(), available_demos[0],
      [](std::string a, std::string b) { return a + ", " + b; }
    ).c_str());
    response->success = false;
    return;
  }

  // Create a request for the convert_to_joint_trajectory service
  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  // Call the convert_to_joint_trajectory service
  // ---------- BEGIN_CITATION [1] ----------
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj](ServiceResponseFuture<ConvertToJointTrajectory> future) {
    auto response = future.get();
    // RCLCPP_INFO(get_logger(), "Received response from convert_to_joint_trajectory service");
    *joint_traj = response->joint_trajectory;

    // Print the joint trajectory
    RCLCPP_INFO(get_logger(), "Joint trajectory created with %ld points:", joint_traj->size());
    // for (unsigned int i = 0; i < joint_traj->size(); i++) {
    //   const auto& joints = joint_traj->at(i);
    //   RCLCPP_INFO(get_logger(), "\t Joint Angles %d: (%.2f, %.2f, %.2f) [rad]", i + 1, joints.theta1, joints.theta2, joints.theta3);
    // }

    RCLCPP_INFO(get_logger(), "Publishing joint trajectory to motors");
    // Publish the joint trajectory to the motors with a 50ms delay between each point
    this->publishMotorCommands(*joint_traj, 50);
  }
  );
  // ---------- END_CITATION [1] ----------

  // Signal success
  response->success = true;
}

std::vector<Point> DeltaMotionPlanner::straightUpDownTrajectory() {
  // Create a simple up down trajectory with 4 oscillations between
  // Z = -100 and Z = -200
  const int num_points = 300;
  std::vector<Point> trajectory;

  const float center = -150.0;
  const float amplitude = 72.0;
  const int cycles = 12;

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = 0.0;
    intermediate_pos.y = 0.0;
    intermediate_pos.z = center + amplitude * sin(2 * M_PI * cycles * t);
    trajectory.push_back(intermediate_pos);
  }

  // Log the created trajectory
  RCLCPP_INFO(get_logger(), "Trajectory created with %ld points:", trajectory.size());
  // for (int i = 0; i < num_points; i++) {
  //   Point p = trajectory[i];
  //   RCLCPP_INFO(get_logger(), "\t EE Point %d: (%.2f, %.2f, %.2f)", i, p.x, p.y, p.z);
  // }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::pringleTrajectory() {
  // Circle Trajectory in XY plane while Z coordinate goes through 2 cycles of a sine wave
  const int num_points = 200;
  const float circle_center_z = -180.0;
  const float amplitude = 25.0;

  std::vector<float> t(num_points);
  float step = (2 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    t[i] = i * step;
  }

  std::vector<float> x_circle(num_points);
  std::vector<float> y_circle(num_points);
  std::vector<float> z_circle(num_points);
  for (int i = 0; i < num_points; ++i) {
    x_circle[i] = (2.0 * amplitude) * cos(t[i]);
    y_circle[i] = (2.0 * amplitude) * sin(t[i]);
    z_circle[i] = circle_center_z + amplitude * sin(2 * t[i]);
  }

  // Create trajectory
  std::vector<Point> trajectory(num_points);
  for (int i = 0; i < num_points; ++i) {
    trajectory[i].x = x_circle[i];
    trajectory[i].y = y_circle[i];
    trajectory[i].z = z_circle[i];
  }

  // Log the created trajectory
  RCLCPP_INFO(get_logger(), "EE Trajectory created with %ld points:", trajectory.size());
  // for (int i = 0; i < num_points; i++) {
  //   Point p = trajectory[i];
  //   RCLCPP_INFO(get_logger(), "\t EE Point %d: (%.2f, %.2f, %.2f)", i + 1, p.x, p.y, p.z);
  // }

  return trajectory;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotionPlanner>());
  rclcpp::shutdown();
  return 0;
}