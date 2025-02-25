#include "delta_motor_control.hpp"

// Control table address for X series(except XL - 320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Converting from degrees to motor position
#define UP_POS 3073.0f // [motor ticks]
#define THETA_MAX (M_PI / 2.0f) // [rad]
#define DOWN_POS 2048.0f // [motor ticks] this value is THETA_MAX in motor ticks
#define RAD_TO_MOTOR_TICKS ((DOWN_POS - UP_POS) / THETA_MAX)

DeltaMotorControl::DeltaMotorControl() : Node("delta_motor_control") {
  RCLCPP_INFO(this->get_logger(), "DeltaMotorControl Started");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  this->portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  this->groupSyncWrite = new dynamixel::GroupSyncWrite(this->portHandler, this->packetHandler, ADDR_GOAL_POSITION, 4);

  this->initializeDynamixels();

  // Subscriber to receive position commands and write them to the motors
  this->delta_joints_sub = this->create_subscription<DeltaJoints>(
    "set_joints",
    QOS_RKL10V,
    [this](const DeltaJoints::SharedPtr msg) -> void
    {
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
        "Motor Positions Set: Motor1: %d, Motor2: %d, Motor3: %d",
        motor_positions[0],
        motor_positions[1],
        motor_positions[2]
      );

    }
  );

  // Service to get the current motor positions
  this->get_positions_server = create_service<GetPositions>(
    "get_motor_positions",
    [this](
      [[maybe_unused]] const std::shared_ptr<GetPositions::Request> request,
      std::shared_ptr<GetPositions::Response> response) -> void
    {
      // Array of Motor Positions
      std::array<int, 3> motor_positions = { 0, 0, 0 };

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

      RCLCPP_INFO(
        this->get_logger(),
        "Motor Positions: (%d, %d, %d)",
        motor_positions[0], motor_positions[1], motor_positions[2]
      );

      response->motor1_position = motor_positions[0];
      response->motor2_position = motor_positions[1];
      response->motor3_position = motor_positions[2];
    }
  );
}

void DeltaMotorControl::initializeDynamixels() {
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // Open Serial Port
  if (this->portHandler->openPort()) {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  if (this->portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
  }

  // Use Position Control Mode
  dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = this->packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
  }
}

uint32_t DeltaMotorControl::convertToMotorPosition(float theta) {
  // Convert theta [rad] to motor position [0, 4096)
  // 2048 is the "zero" position and is when the link is at pi/2 radians (90 deg)
  // 2800 is the "up" position when the link is at 0 radians (0 deg)
  // theta = 0 -> 2800; theta = pi/2 -> 2048; theta = pi/4 -> 2424 (halfway between 2800 and 2048)
  float motor_pos = RAD_TO_MOTOR_TICKS * theta + UP_POS;
  return static_cast<uint32_t>(motor_pos);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotorControl>());
  rclcpp::shutdown();
  return 0;
}