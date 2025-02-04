#ifndef DELTA_KINEMATICS_HPP_
#define DELTA_KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "deltarobot_interfaces/srv/DeltaFK.srv"
#include "deltarobot_interfaces/srv/DeltaIK.srv"


typedef struct DeltaConfig {
  /// @brief Base Triangle Side Length [mm]
  double BL;

  /// @brief Active Link Length [mm]
  double LL;

  /// @brief Passive Link Length [mm]
  double PL;

  /// @brief Passive Link Width [mm]
  double PW;

  /// @brief End Effector Triangle Side Length [mm]
  double EEL;
};


class DeltaKinematics : public rclcpp::Node {
public:
  using DeltaFK = deltarobot_interfaces::srv::DeltaFK;
  using DeltaIK = deltarobot_interfaces::srv::DeltaIK;

  DeltaKinematics();
  ~DeltaKinematics() = default;

private:
  rclcpp::Service<DeltaFK>::SharedPtr delta_fk_server;
  rclcpp::Service<DeltaIK>::SharedPtr delta_ik_server;
  
  void forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response);
  void inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response);
  
  const DeltaConfig robot_config;
};

#endif  // DELTA_KINEMATICS_HPP_