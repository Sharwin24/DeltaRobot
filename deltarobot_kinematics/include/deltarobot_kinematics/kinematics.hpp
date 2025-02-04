#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"


typedef struct {
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
} DeltaConfig;


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
  
  DeltaConfig robot_config;
};

#endif  // KINEMATICS_HPP_