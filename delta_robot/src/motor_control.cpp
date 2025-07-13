/// @file motor_control.cpp
/// @brief Motor Control Implementation for Delta Robot using Dynamixel X-Series Motors
/// @author Sharwin Patil
/// @date 2025-07-13
/// @version 2.0
///
/// PARAMETERS:
///   qos_depth (int8): The depth of the QoS queue for message reliability (default: 10)
///
/// PUBLISHERS:
///   ~/delta_motors/motor_position_feedback (deltarobot_interfaces::msg::DeltaJoints): Publishes real-time motor position feedback in radians
///   ~/delta_motors/motor_velocity_feedback (deltarobot_interfaces::msg::DeltaJointVels): Publishes real-time motor velocity feedback in rad/s
///
/// SUBSCRIBERS:
///   ~/delta_motors/set_joints (deltarobot_interfaces::msg::DeltaJoints): Subscribes to joint position commands to control motor positions
///   ~/delta_motors/set_joint_vels (deltarobot_interfaces::msg::DeltaJointVels): Subscribes to joint velocity commands to control motor velocities
///
/// SERVICES:
///   ~/delta_motors/set_joint_limits (deltarobot_interfaces::srv::SetJointLimits): Service to configure motor position and velocity limits
///
/// CLIENTS:
///   None

#include "motor_control.hpp"

// Control table address for X series(except XL - 320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_MIN_POSITION_LIMIT 48
#define ADDR_MAX_POSITION_LIMIT 52
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_VELOCITY_LIMIT 44
#define ADDR_LED 65

// 3 is for Position Control Mode, 1 is for Velocity Control Mode
#define POSITION_CTRL 3
#define VELOCITY_CTRL 1

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 115200  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Converting from degrees to motor position
#define UP_POS 3073.0f // [motor ticks]
#define THETA_MAX (M_PI / 2.0f) // [rad]
#define DOWN_POS 2048.0f // [motor ticks] this value is THETA_MAX in motor ticks
#define RAD_TO_MOTOR_TICKS ((DOWN_POS - UP_POS) / THETA_MAX)

// Converting from rad/s to rev/min
#define RAD_S_TO_REV_MIN (60.0f * (1.0f / (2.0f * M_PI)))
// The velocity limit range is (0 ~ 1023) where each unit is 0.229 rev/min
#define VEL_UNIT 0.229f // [rev/min]

DeltaMotorControl::DeltaMotorControl() : Node("delta_motor_control") {
  RCLCPP_INFO(this->get_logger(), "DeltaMotorControl Started");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 10;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  this->portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  this->groupSyncWrite = new dynamixel::GroupSyncWrite(this->portHandler, this->packetHandler, ADDR_GOAL_POSITION, 4);

  // Initialize the port, baud rate, control mode
  this->initializeDynamixels();

  // Subscriber to receive position commands and write them to the motors
  this->delta_joints_sub = this->create_subscription<DeltaJoints>(
    "delta_motors/set_joints",
    QOS_RKL10V,
    [this](const DeltaJoints::SharedPtr msg) -> void
  {
    if (this->getControlMode() != POSITION_CTRL) { this->setControlMode(POSITION_CTRL); }
    std::array<uint32_t, 3> motor_positions = {
      convertToMotorPosition(msg->theta1),
      convertToMotorPosition(msg->theta2),
      convertToMotorPosition(msg->theta3)
    };

    // Clear the groupSyncWrite data
    this->groupSyncWrite->clearParam();

    // Position Value of X series is 4 byte data.
    // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    for (uint8_t i = 0; i < motor_positions.size(); i++) {
      // Create parameter for GroupSyncWrite
      uint8_t param_goal_position[4];
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_positions[i]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_positions[i]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_positions[i]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_positions[i]));

      if (!this->groupSyncWrite->addParam(i + 1, param_goal_position)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add param to groupSyncWrite");
      }
    }

    // Transmit all position commands at once
    int dxl_comm_result = this->groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "GroupSyncWrite failed: %s", this->packetHandler->getTxRxResult(dxl_comm_result));
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Motor Positions Set: (%d, %d, %d) [rad]",
      motor_positions[0],
      motor_positions[1],
      motor_positions[2]
    );
  }
  );

  this->delta_joint_vels_sub = this->create_subscription<DeltaJointVels>(
    "delta_motors/set_joint_vels",
    QOS_RKL10V,
    [this](const DeltaJointVels::SharedPtr msg) -> void
  {
    if (this->getControlMode() != VELOCITY_CTRL) { this->setControlMode(VELOCITY_CTRL); }
    std::array<int, 3> motor_vels = {
      this->convertToMotorVelocity(msg->theta1_vel),
      this->convertToMotorVelocity(msg->theta2_vel),
      this->convertToMotorVelocity(msg->theta3_vel)
    };
    RCLCPP_INFO(
      this->get_logger(),
      "Motor Velocities Set: (%d, %d, %d) [rev/min]",
      motor_vels[0],
      motor_vels[1],
      motor_vels[2]
    );

    // Clear the groupSyncWrite data
    this->groupSyncWrite->clearParam();

    // Velocity Value of X series is 4 byte data.
    // For AX & MX(1.0) use 2 byte data(uint16_t) for the Velocity Value.
    for (uint8_t i = 0; i < 3; i++) {
      // Create parameter for GroupSyncWrite
      uint8_t param_goal_velocity[4];
      param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(motor_vels[i]));
      param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(motor_vels[i]));
      param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(motor_vels[i]));
      param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(motor_vels[i]));

      if (!this->groupSyncWrite->addParam(i + 1, param_goal_velocity)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add param to groupSyncWrite");
      }
    }

    // Transmit all velocity commands at once
    int dxl_comm_result = this->groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "GroupSyncWrite failed: %s", this->packetHandler->getTxRxResult(dxl_comm_result));
    }

    /**
     * Manually publish zero velocity afterwards to stop the motors
     */
    this->groupSyncWrite->clearParam();
    for (uint8_t i = 0; i < 3; i++) {
      // Create parameter for GroupSyncWrite
      uint8_t param_goal_velocity[4];
      param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(0));
      param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(0));
      param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(0));
      param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(0));

      if (!this->groupSyncWrite->addParam(i + 1, param_goal_velocity)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add param to groupSyncWrite");
      }
    }

    // Transmit the stop command
    dxl_comm_result = this->groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "GroupSyncWrite failed: %s", this->packetHandler->getTxRxResult(dxl_comm_result));
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Motor Velocities Set: (%f, %f, %f) [rad/s]",
      msg->theta1_vel,
      msg->theta2_vel,
      msg->theta3_vel
    );
  }
  );

  // Service to set the joint limits
  this->set_joint_limits_server = create_service<SetJointLimits>(
    "delta_motors/set_joint_limits",
    [this](
      const std::shared_ptr<SetJointLimits::Request> request,
      std::shared_ptr<SetJointLimits::Response> response) -> void
  {
    // Convert request data into motor position and velocity values
    uint32_t motor_min = this->convertToMotorPosition(request->min_rad);
    uint32_t motor_max = this->convertToMotorPosition(request->max_rad);
    int max_vel = this->convertToMotorVelocity(request->max_vel_rad_s);
    int dxl_comm_result = COMM_TX_FAIL;

    this->disableTorque();

    // Set Position Limits on MIN_POSITION_LIMIT and MAX_POSITION_LIMIT
    // Set Velocity Limit on VELOCITY_LIMIT
    for (uint8_t i = 1; i <= 3; i++) {
      uint8_t dxl_error = 0;
      dxl_comm_result = this->packetHandler->write4ByteTxRx(
        this->portHandler,
        i,
        ADDR_MIN_POSITION_LIMIT,
        motor_min,
        &dxl_error
      );
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Min Position Limit: %d", dxl_error);
      }

      dxl_comm_result = this->packetHandler->write4ByteTxRx(
        this->portHandler,
        i,
        ADDR_MAX_POSITION_LIMIT,
        motor_max,
        &dxl_error
      );
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Max Position Limit: %d", dxl_error);
      }

      dxl_comm_result = this->packetHandler->write4ByteTxRx(
        this->portHandler,
        i,
        ADDR_VELOCITY_LIMIT,
        max_vel,
        &dxl_error
      );
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity Limit: %d", dxl_error);
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Joint Limits Set: Position [%f, %f] [rad], Velocity: %.2f [rad/s]", request->min_rad, request->max_rad, request->max_vel_rad_s
    );

    this->enableTorque();
    response->success = true;
  }
  );

  // Publishers for motor positions and velocities
  this->motor_positions_pub = this->create_publisher<DeltaJoints>("delta_motors/motor_position_feedback", 10);
  this->motor_velocities_pub = this->create_publisher<DeltaJointVels>("delta_motors/motor_velocity_feedback", 10);

  // Timer to publish the motor positions and velocities
  const double feedback_freq = 25.0; // [Hz]
  this->motor_feedback_timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / feedback_freq),
    [this]() -> void
  {
    // Array of Motor Positions
    std::array<int, 3> motor_positions = {0, 0, 0};

    for (uint8_t i = 1; i <= motor_positions.size(); i++) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = COMM_TX_FAIL;
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = this->packetHandler->read4ByteTxRx(
        this->portHandler,
        i,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t*>(&motor_positions[i - 1]),
        &dxl_error
      );
      // Error Handling
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getRxPacketError(dxl_error));
      }
    }

    // Convert positions from dynamixel units to radians
    float theta1 = convertToRadians(motor_positions[0]);
    float theta2 = convertToRadians(motor_positions[1]);
    float theta3 = convertToRadians(motor_positions[2]);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Motor Positions: (%d, %d, %d) [motor ticks] -> (%f, %f, %f) [rad]",
      motor_positions[0], motor_positions[1], motor_positions[2],
      theta1, theta2, theta3
    );

    // Array of Motor Velocities
    std::array<int, 3> motor_velocities = {0, 0, 0};

    for (uint8_t i = 1; i <= motor_velocities.size(); i++) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = COMM_TX_FAIL;
      // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = this->packetHandler->read4ByteTxRx(
        this->portHandler,
        i,
        ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t*>(&motor_velocities[i - 1]),
        &dxl_error
      );
      // Error Handling
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getRxPacketError(dxl_error));
      }
    }

    // Convert velocities from dynamixel units to rev/min
    for (uint8_t i = 0; i < motor_velocities.size(); i++) {
      motor_velocities[i] = motor_velocities[i] * VEL_UNIT;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Motor Velocities: (%d, %d, %d) [rev/min]",
      motor_velocities[0], motor_velocities[1], motor_velocities[2]
    );

    // Publish the motor positions and velocities
    auto motor_positions_msg = DeltaJoints();
    auto motor_velocities_msg = DeltaJointVels();
    motor_positions_msg.header.stamp = this->now();
    motor_velocities_msg.header.stamp = this->now();
    motor_positions_msg.theta1 = theta1;
    motor_positions_msg.theta2 = theta2;
    motor_positions_msg.theta3 = theta3;
    motor_velocities_msg.theta1_vel = motor_velocities[0];
    motor_velocities_msg.theta2_vel = motor_velocities[1];
    motor_velocities_msg.theta3_vel = motor_velocities[2];

    this->motor_positions_pub->publish(motor_positions_msg);
    this->motor_velocities_pub->publish(motor_velocities_msg);
  }
  );
}

DeltaMotorControl::~DeltaMotorControl() {
  this->disableTorque();
  // Turn off the LED
  uint8_t dxl_error = 0;
  this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_LED,
    0,
    &dxl_error
  );
  this->portHandler->closePort();
  delete this->portHandler;
  delete this->packetHandler;
  delete this->groupSyncWrite;

}

void DeltaMotorControl::setControlMode(uint8_t ctrl_mode) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Set the Operating Mode
  dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_OPERATING_MODE,
    ctrl_mode,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Operating Mode: %d", dxl_error);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set Operating Mode (%s)", (ctrl_mode == POSITION_CTRL) ? "Position Control" : "Velocity Control");
  }

  // Save the control mode
  this->control_mode = ctrl_mode;
}

void DeltaMotorControl::initializeDynamixels() {
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // Open Serial Port
  if (this->portHandler->openPort()) {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port (%s)", this->portHandler->getPortName());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  if (this->portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate (%d)", this->portHandler->getBaudRate());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
  }

  this->setControlMode(POSITION_CTRL);

  // Set LED of DYNAMIXEL
  dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_LED,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set LED.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set Motor LED.");
  }

  this->enableTorque();
}

void DeltaMotorControl::disableTorque() {
  uint8_t dxl_error = 0;
  // Disable the torque before changing the control mode
  int dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to disable torque: %d", dxl_error);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to disable torque.");
  }
}

void DeltaMotorControl::enableTorque() {
  uint8_t dxl_error = 0;
  // Enable the torque after changing the control mode
  int dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque: %d", dxl_error);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
  }
}

float DeltaMotorControl::convertToRadians(int motor_pos) {
  // Convert motor position [0, 4096) to theta [rad]
  return (motor_pos - UP_POS) / RAD_TO_MOTOR_TICKS;
}

uint32_t DeltaMotorControl::convertToMotorPosition(float theta) {
  // Convert theta [rad] to motor position [0, 4096)
  // 2048 is the "zero" position and is when the link is at pi/2 radians (90 deg)
  // 2800 is the "up" position when the link is at 0 radians (0 deg)
  // theta = 0 -> 2800; theta = pi/2 -> 2048; theta = pi/4 -> 2424 (halfway between 2800 and 2048)
  float motor_pos = RAD_TO_MOTOR_TICKS * theta + UP_POS;
  return static_cast<uint32_t>(motor_pos);
}

int DeltaMotorControl::convertToMotorVelocity(float theta_vel) {
  // Convert theta_vel [rad/s] to motor velocity [rev/min]
  float rpm = RAD_S_TO_REV_MIN * theta_vel;
  // Convert motor velocity [rev/min] to the appropriate value
  return static_cast<int>(rpm / VEL_UNIT);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotorControl>());
  rclcpp::shutdown();
  return 0;
}