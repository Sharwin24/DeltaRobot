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

DeltaMotorControl::DeltaMotorControl() : Node("delta_motor_control") {
  RCLCPP_INFO(this->get_logger(), "DeltaMotorControl Started");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  this->portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // Open Serial Port
  dxl_comm_result = this->portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = this->portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");
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
  }
  else {
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
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
  }

  // Subscriber to receive position commands and write them to the motors
  this->position_cmd_subscriber =
    this->create_subscription<PositionCmd>(
      "set_motor_positions",
      QOS_RKL10V,
      [this](const PositionCmd::SharedPtr msg) -> void
      {
        // Position Value of X series is 4 byte data.
        // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
        // Motor positions Array
        std::array<uint32_t, 3> motor_positions = { (uint32_t)msg->motor1_pos, (uint32_t)msg->motor2_pos, (uint32_t)msg->motor3_pos };

        for (uint8_t i = 1; i <= motor_positions.size(); i++) {
          uint8_t dxl_error = 0;
          int dxl_comm_result = COMM_TX_FAIL;
          // Write Motor position (length : 4 bytes)
          dxl_comm_result =
            this->packetHandler->write4ByteTxRx(
              this->portHandler,
              i,
              ADDR_GOAL_POSITION,
              motor_positions[i - 1],
              &dxl_error
            );
          // Error Handling
          if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getTxRxResult(dxl_comm_result));
          }
          else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getRxPacketError(dxl_error));
          }
        }
        
        RCLCPP_DEBUG(
          this->get_logger(),
          "Motor Positions Set: Motor1: %d, Motor2: %d, Motor3: %d",
          msg->motor1_pos,
          msg->motor2_pos,
          msg->motor3_pos
        );

      }
    );
  
  // Service to get the current motor positions
  auto get_positions =
    [this](
      [[maybe_unused]] const std::shared_ptr<GetPositions::Request> request,
      std::shared_ptr<GetPositions::Response> response) -> void
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
        }
        else if (dxl_error != 0) {
          RCLCPP_INFO(this->get_logger(), "%s", this->packetHandler->getRxPacketError(dxl_error));
        }
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Motor Positions: Motor1: %d, Motor2: %d, Motor3: %d",
        motor_positions[0], motor_positions[1], motor_positions[2]
      );

      response->motor1_position = motor_positions[0];
      response->motor2_position = motor_positions[1];
      response->motor3_position = motor_positions[2];
    };

  this->get_positions_server = create_service<GetPositions>("get_motor_positions", get_positions);

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotorControl>());
  rclcpp::shutdown();
  return 0;
}