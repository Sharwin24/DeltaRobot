/// @file motion_planner.cpp
/// @brief Motion Planning Implementation for Delta Robot
/// @author Sharwin Patil
/// @date 2025-07-13
/// @version 2.0
///
/// PARAMETERS:
///   None
///
/// PUBLISHERS:
///   ~/delta_motors/set_joints (deltarobot_interfaces::msg::DeltaJoints): Publishes joint position commands to the delta robot motors
///   ~/delta_motors/set_joint_vels (deltarobot_interfaces::msg::DeltaJointVels): Publishes joint velocity commands to the delta robot motors
///
/// SUBSCRIBERS:
///   None
///
/// SERVICES:
///   ~/delta_motion_planner/play_demo_trajectory (deltarobot_interfaces::srv::PlayDemoTrajectory): Service to play predefined demonstration trajectories (up_down, pringle, axes, circle, scan)
///   ~/delta_motion_planner/move_to_point (deltarobot_interfaces::srv::MoveToPoint): Service to move the end effector to a specific 3D point
///   ~/delta_motion_planner/move_to_configuration (deltarobot_interfaces::srv::MoveToConfiguration): Service to move the robot to a specific joint configuration
///   ~/delta_motion_planner/motion_demo (deltarobot_interfaces::srv::MotionDemo): Service to start/stop the automatic demo mode
///
/// CLIENTS:
///   ~/delta_kinematics/delta_ik (deltarobot_interfaces::srv::DeltaIK): Client to compute inverse kinematics (joint angles from end effector position)
///   ~/delta_kinematics/delta_fk (deltarobot_interfaces::srv::DeltaFK): Client to compute forward kinematics (end effector position from joint angles)
///   ~/delta_kinematics/convert_to_joint_trajectory (deltarobot_interfaces::srv::ConvertToJointTrajectory): Client to convert end effector trajectory to joint trajectory
///   ~/delta_kinematics/convert_to_joint_vel_trajectory (deltarobot_interfaces::srv::ConvertToJointVelTrajectory): Client to convert end effector velocity trajectory to joint velocity trajectory

#include "rclcpp/rclcpp.hpp"
#include "motion_planner.hpp"

// Include all the custom messages and services
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>
#include <fstream>

template<typename T>
using ServiceResponseFuture = typename rclcpp::Client<T>::SharedFuture;

using Point = geometry_msgs::msg::Point;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using PlayDemoTraj = deltarobot_interfaces::srv::PlayDemoTrajectory;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using ConvertToJointVelTrajectory = deltarobot_interfaces::srv::ConvertToJointVelTrajectory;

DeltaMotionPlanner::DeltaMotionPlanner() : Node("delta_motion_planner") {
  RCLCPP_INFO(get_logger(), "DeltaMotionPlanner node started");

  this->demo_traj_server = create_service<PlayDemoTraj>(
    "delta_motion_planner/play_demo_trajectory",
    std::bind(&DeltaMotionPlanner::playDemoTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  this->joint_pub = this->create_publisher<DeltaJoints>("delta_motors/set_joints", QOS_RKL10V);
  this->joint_vel_pub = this->create_publisher<DeltaJointVels>("delta_motors/set_joint_vels", QOS_RKL10V);

  this->delta_ik_client = create_client<DeltaIK>("delta_kinematics/delta_ik");
  while (!this->delta_ik_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->delta_fk_client = create_client<DeltaFK>("delta_kinematics/delta_fk");
  while (!this->delta_fk_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->convert_to_joint_trajectory_client = create_client<ConvertToJointTrajectory>("delta_kinematics/convert_to_joint_trajectory");
  while (!this->convert_to_joint_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  this->convert_to_joint_vel_trajectory_client = create_client<ConvertToJointVelTrajectory>("delta_kinematics/convert_to_joint_vel_trajectory");
  while (!this->convert_to_joint_vel_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  // Create service for MoveToPoint
  this->move_to_point_server = create_service<MoveToPoint>(
    "delta_motion_planner/move_to_point",
    [this](const std::shared_ptr<MoveToPoint::Request> request, std::shared_ptr<MoveToPoint::Response> response) {
    Point point = request->target;
    this->moveToPoint(point);
    response->success = true;
  }
  );

  // Create service for MoveToConfiguration
  this->move_to_configuration_server = create_service<MoveToConfiguration>(
    "delta_motion_planner/move_to_configuration",
    [this](const std::shared_ptr<MoveToConfiguration::Request> request, std::shared_ptr<MoveToConfiguration::Response> response) {
    DeltaJoints joints = request->target_joint_angles;
    this->moveToConfiguration(joints);
    response->success = true;
  }
  );

  // Create service for MotionDemo
  this->motion_demo_server = create_service<MotionDemo>(
    "delta_motion_planner/motion_demo",
    [this](const std::shared_ptr<MotionDemo::Request> request, [[maybe_unused]] std::shared_ptr<MotionDemo::Response> response)
  {this->playDemo = request->start;}
  );

  const float demoDelay = 32; // Every demoDelay seconds, run the MSI demo trajectory
  this->demo_timer = this->create_wall_timer(
    std::chrono::duration<float>(demoDelay),
    [this]() -> void {
    if (this->playDemo) {
      RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory");

      this->playTrajectory(this->pringleTrajectory());
      this->playTrajectory(this->circleTrajectory());
      this->playTrajectory(this->axesTrajectory());
      this->playTrajectory(this->randomSampleTrajectory(20));
    }
  }
  );
}

void DeltaMotionPlanner::publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms) {
  // Publish the joint trajectory to the motors with a small delay [ms] between each point
  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    this->joint_pub->publish(joint_traj[i]);
    rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

void DeltaMotionPlanner::publishMotorVelocityCommands(const std::vector<DeltaJointVels>& joint_vel_traj, const unsigned int delay_ms) {
  // Publish the joint velocity trajectory to the motors with a small delay [ms] between each point
  for (unsigned int i = 0; i < joint_vel_traj.size(); i++) {
    this->joint_vel_pub->publish(joint_vel_traj[i]);
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

void DeltaMotionPlanner::moveThroughPoints(const std::vector<Point>& points) {
  // Plan a continuous trajectory through the given points using 3rd order polynomial interpolation
  (void)points;
}

void DeltaMotionPlanner::playTrajectory(const std::vector<Point> trajectory) {
  // Create a joint trajectory using the convert_to_joint_trajectory service
  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  // ---------- BEGIN_CITATION ----------
  // https://github.com/ros2/rclcpp/issues/312
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj](ServiceResponseFuture<ConvertToJointTrajectory> future) {
    auto response = future.get();
    *joint_traj = response->joint_trajectory;

    // Publish the joint trajectory to the motors with small delay
    this->publishMotorCommands(*joint_traj, 50);
  }
  );
  // ---------- END_CITATION ----------
}

void DeltaMotionPlanner::playDemoTrajectory(
  std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response) {

  std::string type = request->type.data;
  std::vector<Point> trajectory;
  const std::vector<std::string> available_demos = {"up_down", "pringle", "axes", "circle", "scan"};
  if (type == "up_down") {
    trajectory = this->straightUpDownTrajectory();
  } else if (type == "pringle") {
    trajectory = this->pringleTrajectory();
  } else if (type == "axes") {
    trajectory = this->axesTrajectory();
  } else if (type == "circle") {
    trajectory = this->circleTrajectory();
  } else if (type == "scan") {
    trajectory = this->scanTrajectory();
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid demo trajectory: %s", type.c_str());
    RCLCPP_ERROR(get_logger(), "Available demo trajectories: %s", std::accumulate(
      std::next(available_demos.begin()), available_demos.end(), available_demos[0],
      [](std::string a, std::string b) { return a + ", " + b; }
    ).c_str());
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Playing demo trajectory: %s", type.c_str());

  this->playTrajectory(trajectory);

  // Signal success
  response->success = true;
}

std::vector<Point> DeltaMotionPlanner::scanTrajectory() {
  // Scan trajectory is saved in "scan_trajectory.csv" file
  return this->readCSV("csv_files/scan_trajectory.csv");
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

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::axesTrajectory() {
  // Trajectory showcasing the DOF of the DeltaRobot
  // Path will be a translation along X axis, then Y axis, then Z axis

  std::vector<Point> trajectory;

  const float x_start = 0.0;
  const float x_end = 60.0;
  const float y_start = 0.0;
  const float y_end = 60.0;
  const float z_start = -180.0;
  const float z_end = -240.0;
  const int num_points = 25;

  // X Axis Translation from (0, 0, -180) to (80, 0, -180)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start + t * (x_end - x_start);
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_end - t * (x_end - x_start);
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }

  // Y Axis translation from (0, 0, -180) to (0, 80, -180)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start + t * (y_end - y_start);
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_end - t * (y_end - y_start);
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }

  // Z Axis translation from (0, 0, -180) to (0, 0, -220)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start + t * (z_end - z_start);
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_end - t * (z_end - z_start);
    trajectory.push_back(intermediate_pos);
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::circleTrajectory() {
  // Circle Trajectory in XY plane while Z coordinate remains constant
  const int num_points = 200;
  const float center_z = -180.0;
  const float radius = 40.0;

  std::vector<float> t(num_points);
  float step = (2 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    t[i] = i * step;
  }

  std::vector<float> x_circle(num_points);
  std::vector<float> y_circle(num_points);
  std::vector<float> z_circle(num_points);
  for (int i = 0; i < num_points; ++i) {
    x_circle[i] = radius * cos(t[i]);
    y_circle[i] = radius * sin(t[i]);
    z_circle[i] = center_z;
  }

  // Create trajectory
  std::vector<Point> trajectory(num_points);
  for (int i = 0; i < num_points; ++i) {
    trajectory[i].x = x_circle[i];
    trajectory[i].y = y_circle[i];
    trajectory[i].z = z_circle[i];
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::randomSampleTrajectory(const int numPoints) {
  // Get all points from a random sample from random_points.csv
  std::vector<Point> allPoints = this->readCSV("csv_files/random_points.csv");

  // Randomly sample numPoints points from allPoints
  std::vector<Point> sampledPoints;
  std::srand(std::time(0)); // Seed for random number generation
  for (int i = 0; i < numPoints; ++i) {
    int randomIndex = std::rand() % allPoints.size();
    sampledPoints.push_back(allPoints[randomIndex]);
  }
  return sampledPoints;
}

std::vector<Point> DeltaMotionPlanner::readCSV(const std::string& fileName) {
  const std::string user = std::getenv("USER");
  const std::string file_path = "/home/" + user + "/DeltaRobot/" + fileName;
  // The csv file has 3 columns: X, Y, Z
  std::ifstream file(fileName);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
    return {};
  }
  std::vector<Point> trajectory;
  std::string line;
  bool first_line = true; // Skip the header
  while (std::getline(file, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }
    std::istringstream iss(line);
    std::string value;
    Point p;

    std::getline(iss, value, ',');
    p.x = std::stod(value);
    std::getline(iss, value, ',');
    p.y = std::stod(value);
    std::getline(iss, value, ',');
    p.z = std::stod(value);

    trajectory.push_back(p);
  }
  file.close();
  return trajectory;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotionPlanner>());
  rclcpp::shutdown();
  return 0;
}