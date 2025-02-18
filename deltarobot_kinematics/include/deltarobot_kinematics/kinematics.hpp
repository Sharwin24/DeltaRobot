#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include <math.h>

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

  // Helper function for FK to find active link angle when normal to the link's rotation axis (YZ-plane)
  int deltaFK_AngleYZ(float x0, float y0, float z0, float& theta);
  
  /// @brief Base Triangle Side Length [mm]
  float SB;

  /// @brief End Effector Platform Triangle Side Length [mm]
  float SP;

  /// @brief Active Link Length [mm]
  float AL;

  /// @brief Passive Link Length [mm]
  float PL;

  /// @brief Passive Link Width [mm]
  float PW;
};

#endif  // KINEMATICS_HPP_