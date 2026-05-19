#ifndef AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_
#define AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

#include "canopen_ros2_control/robot_system.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace ar_project
{

class EmbodiedRobotSystem : public canopen_ros2_control::RobotSystem
{
public:
  EmbodiedRobotSystem() = default;
  ~EmbodiedRobotSystem() override = default;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool switch_operation_mode_via_sdo(
    canopen_ros2_control::Cia402Data & motor, uint16_t mode, const std::string & reason);

  bool read_u32_via_sdo(
    canopen_ros2_control::Cia402Data & motor, uint16_t index, uint8_t subindex, uint32_t & value);

  void poll_and_log_fault_state(canopen_ros2_control::Cia402Data & motor);

  double joint_direction_sign(const std::string & joint_name) const;

  double velocity_scale_for_joint(const std::string & joint_name);

  std::unordered_map<std::string, double> velocity_scale_to_dev_;
  std::unordered_set<std::string> active_faults_;
  size_t fault_poll_counter_ = 0;
};

}  // namespace ar_project

#endif  // AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_
