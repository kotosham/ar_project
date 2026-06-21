#ifndef AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_
#define AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_

#include <atomic>
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

  // Returns true if the joint is currently in an EPOS4 fault state. Logs on
  // fault entry/exit transitions (Phase 0.3).
  bool poll_fault_state(canopen_ros2_control::Cia402Data & motor);

  // Latch a quick-stop request. RT-safe: only flips an atomic flag and logs
  // once. The actual CiA-402 quick-stop is emitted from write() (Phase 0.2).
  void request_quick_stop(const std::string & reason);

  // Emit a CiA-402 quick-stop on the RT write() path via the controlword RPDO
  // (no blocking SDO). Also commands zero target velocity defensively.
  bool transmit_quick_stop(canopen_ros2_control::Cia402Data & motor);

  double joint_direction_sign(const std::string & joint_name);

  double velocity_scale_for_joint(const std::string & joint_name);

  std::unordered_map<std::string, double> joint_direction_signs_;
  std::unordered_map<std::string, double> velocity_scale_to_dev_;
  std::unordered_set<std::string> active_faults_;
  size_t fault_poll_counter_ = 0;

  // Poll the statusword once every N read() cycles. At a 50 Hz controller this
  // is a 10 Hz fault scan (<=100 ms detection), well inside the <200 ms stop
  // budget, without flooding the time-sensitive CAN bus with SDO traffic.
  // NOTE (build-env follow-up): the statusword 0x6041 is already mapped into
  // TPDO1, so a true per-cycle, non-blocking scan should read the driver's
  // cached TPDO value instead of an SDO; promote once the canopen_ros2_control
  // accessor is confirmed on the target. Until then we keep the confirmed SDO
  // path and decimate it.
  size_t fault_poll_decimation_ = 5;

  // Latched once a quick-stop is requested (drive fault today; external Stop
  // action / collision / bus-off will also set it in later phases). Atomic so a
  // future non-RT trigger can set it safely.
  std::atomic<bool> quick_stop_active_{false};
};

}  // namespace ar_project

#endif  // AR_PROJECT__EMBODIED_ROBOT_SYSTEM_HPP_
