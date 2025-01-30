
#include "delta_kinematics.hpp"


DeltaKinematics::DeltaKinematics() : Node("delta_kinematics") {
  RCLCPP_INFO(this->get_logger(), "DeltaKinematics Started");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  delta_fk_server = create_service<DeltaFK>("delta_fk", this->forwardKinematics);
  delta_ik_server = create_service<DeltaIK>("delta_ik", this->inverseKinematics);
}

DeltaKinematics::~DeltaKinematics() {}

void DeltaKinematics::forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response) {

}
void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {

}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}