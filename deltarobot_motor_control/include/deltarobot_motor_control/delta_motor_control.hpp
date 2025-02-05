#ifndef DELTA_MOTOR_CONTROL_HPP_
#define DELTA_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "deltarobot_interfaces/msg/position_cmd.hpp"
#include "deltarobot_interfaces/srv/get_positions.hpp"


class DeltaMotorControl : public rclcpp::Node {
public:
  using PositionCmd = deltarobot_interfaces::msg::PositionCmd;
  using GetPositions = deltarobot_interfaces::srv::GetPositions;

  DeltaMotorControl();
  ~DeltaMotorControl() = default;  

private:
  rclcpp::Subscription<PositionCmd>::SharedPtr position_cmd_subscriber;
  rclcpp::Service<GetPositions>::SharedPtr get_positions_server;

  dynamixel::PortHandler* portHandler;
  dynamixel::PacketHandler* packetHandler;

};

#endif  // DELTA_MOTOR_CONTROL_HPP_