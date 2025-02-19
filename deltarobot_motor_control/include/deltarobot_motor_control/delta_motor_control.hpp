#ifndef DELTA_MOTOR_CONTROL_HPP_
#define DELTA_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/get_dynamixel_positions.hpp"


class DeltaMotorControl : public rclcpp::Node {
public:
  using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
  using GetPositions = deltarobot_interfaces::srv::GetDynamixelPositions;

  DeltaMotorControl();
  ~DeltaMotorControl() = default;

  
  private:
  rclcpp::Subscription<DeltaJoints>::SharedPtr delta_joints_sub;
  rclcpp::Service<GetPositions>::SharedPtr get_positions_server;
  
  dynamixel::PortHandler* portHandler;
  dynamixel::PacketHandler* packetHandler;
  
  uint32_t convertToMotorPosition(float theta);
};

#endif  // DELTA_MOTOR_CONTROL_HPP_