#include "ar_project/embodied_robot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_core/exchange.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace ar_project
{

bool EmbodiedRobotSystem::switch_operation_mode_via_sdo(
  canopen_ros2_control::Cia402Data & motor, uint16_t mode, const std::string & reason)
{
  ros2_canopen::COData write_mode = {0x6060, 0x00, static_cast<uint32_t>(mode)};
  if (!motor.driver->sdo_write(write_mode))
  {
    RCLCPP_ERROR(
      robot_system_logger,
      "Failed to write operation mode '%u' to joint '%s' during %s.",
      static_cast<unsigned>(mode), motor.joint_name.c_str(), reason.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  ros2_canopen::COData read_mode = {0x6061, 0x00, 0U};
  if (!motor.driver->sdo_read(read_mode))
  {
    RCLCPP_ERROR(
      robot_system_logger,
      "Failed to read back operation mode display for joint '%s' during %s.",
      motor.joint_name.c_str(), reason.c_str());
    return false;
  }

  const auto applied_mode = static_cast<uint8_t>(read_mode.data_ & 0xFF);
  if (applied_mode != static_cast<uint8_t>(mode))
  {
    RCLCPP_ERROR(
      robot_system_logger,
      "Joint '%s' reported operation mode display '%u' instead of requested '%u' during %s.",
      motor.joint_name.c_str(), static_cast<unsigned>(applied_mode), static_cast<unsigned>(mode),
      reason.c_str());
    return false;
  }

  RCLCPP_INFO(
    robot_system_logger,
    "Joint '%s' confirmed operation mode '%u' via SDO during %s.",
    motor.joint_name.c_str(), static_cast<unsigned>(applied_mode), reason.c_str());
  return true;
}

bool EmbodiedRobotSystem::read_u32_via_sdo(
  canopen_ros2_control::Cia402Data & motor, uint16_t index, uint8_t subindex, uint32_t & value)
{
  ros2_canopen::COData data = {index, subindex, 0U};
  if (!motor.driver->sdo_read(data))
  {
    return false;
  }

  value = data.data_;
  return true;
}

void EmbodiedRobotSystem::poll_and_log_fault_state(canopen_ros2_control::Cia402Data & motor)
{
  uint32_t statusword = 0U;
  if (!read_u32_via_sdo(motor, 0x6041, 0x00, statusword))
  {
    return;
  }

  const bool in_fault = (statusword & 0x0008U) != 0U;
  const bool was_in_fault = active_faults_.find(motor.joint_name) != active_faults_.end();

  if (in_fault && !was_in_fault)
  {
    uint32_t error_code = 0U;
    uint32_t error_register = 0U;
    uint32_t error_count = 0U;
    uint32_t last_error = 0U;

    const bool has_error_code = read_u32_via_sdo(motor, 0x603F, 0x00, error_code);
    const bool has_error_register = read_u32_via_sdo(motor, 0x1001, 0x00, error_register);
    const bool has_error_count = read_u32_via_sdo(motor, 0x1003, 0x00, error_count);
    const bool has_last_error =
      has_error_count && (error_count & 0xFFU) > 0U &&
      read_u32_via_sdo(motor, 0x1003, 0x01, last_error);

    RCLCPP_ERROR(
      robot_system_logger,
      "Joint '%s' entered EPOS4 fault state: statusword=0x%04x, error_code=%s0x%04x, "
      "error_register=%s0x%02x, error_history_count=%s%u, last_error=%s0x%08x.",
      motor.joint_name.c_str(),
      static_cast<unsigned>(statusword & 0xFFFFU),
      has_error_code ? "" : "unavailable/",
      has_error_code ? static_cast<unsigned>(error_code & 0xFFFFU) : 0U,
      has_error_register ? "" : "unavailable/",
      has_error_register ? static_cast<unsigned>(error_register & 0xFFU) : 0U,
      has_error_count ? "" : "unavailable/",
      has_error_count ? static_cast<unsigned>(error_count & 0xFFU) : 0U,
      has_last_error ? "" : "unavailable/",
      has_last_error ? static_cast<unsigned>(last_error) : 0U);

    active_faults_.insert(motor.joint_name);
  }
  else if (!in_fault && was_in_fault)
  {
    RCLCPP_INFO(
      robot_system_logger,
      "Joint '%s' cleared EPOS4 fault state.",
      motor.joint_name.c_str());
    active_faults_.erase(motor.joint_name);
  }
}

double EmbodiedRobotSystem::joint_direction_sign(const std::string & joint_name) const
{
  // The real hardware sign convention differs from the nominal URDF convention on the left side.
  // Applying the correction here keeps diff_drive_controller in joint space while commanding the
  // EPOS4s with the raw sign each motor expects.
  if (joint_name == "left_wheel_joint")
  {
    return -1.0;
  }

  return 1.0;
}

double EmbodiedRobotSystem::velocity_scale_for_joint(const std::string & joint_name)
{
  const auto cached = velocity_scale_to_dev_.find(joint_name);
  if (cached != velocity_scale_to_dev_.end())
  {
    return cached->second;
  }

  double scale = 1.0;
  try
  {
    const auto bus_config = YAML::LoadFile(bus_config_);

    if (bus_config["defaults"] && bus_config["defaults"]["scale_vel_to_dev"])
    {
      scale = bus_config["defaults"]["scale_vel_to_dev"].as<double>();
    }

    if (
      bus_config["nodes"] && bus_config["nodes"][joint_name] &&
      bus_config["nodes"][joint_name]["scale_vel_to_dev"])
    {
      scale = bus_config["nodes"][joint_name]["scale_vel_to_dev"].as<double>();
    }

    // Installed/generated bus.yml is flattened, with one top-level key per joint.
    if (bus_config[joint_name] && bus_config[joint_name]["scale_vel_to_dev"])
    {
      scale = bus_config[joint_name]["scale_vel_to_dev"].as<double>();
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(
      robot_system_logger,
      "Failed to read scale_vel_to_dev for joint '%s' from '%s': %s. Falling back to 1.0.",
      joint_name.c_str(), bus_config_.c_str(), e.what());
  }

  velocity_scale_to_dev_[joint_name] = scale;
  return scale;
}

hardware_interface::CallbackReturn EmbodiedRobotSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(
    robot_system_logger,
    "Activating CANopen hardware with EPOS4 enable/recover path instead of init+homing.");

  for (auto & motor : robot_motor_data_)
  {
    if (!motor.driver)
    {
      RCLCPP_ERROR(
        robot_system_logger, "Joint '%s' has no bound Cia402Driver.", motor.joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    motor.driver->start_node_nmt_command();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  for (auto & motor : robot_motor_data_)
  {
    if (!motor.driver->recover_motor())
    {
      RCLCPP_ERROR(
        robot_system_logger,
        "Recover/enable failed for joint '%s'.",
        motor.joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto velocity_it =
      motor.command_interface_to_operation_mode.find(
      motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    if (velocity_it != motor.command_interface_to_operation_mode.end())
    {
      if (!switch_operation_mode_via_sdo(motor, velocity_it->second, "hardware activation"))
      {
        RCLCPP_ERROR(
          robot_system_logger,
          "Failed to switch joint '%s' into configured velocity mode '%u'.",
          motor.joint_name.c_str(),
          static_cast<unsigned>(velocity_it->second));
        return hardware_interface::CallbackReturn::ERROR;
      }

      motor.interfaces_to_running = {
        motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY};
      motor.interfaces_to_start.clear();
      motor.interfaces_to_stop.clear();

      RCLCPP_INFO(
        robot_system_logger,
        "Joint '%s' is enabled and set to velocity mode '%u'.",
        motor.joint_name.c_str(),
        static_cast<unsigned>(velocity_it->second));
    }
  }

  RCLCPP_INFO(robot_system_logger, "EPOS4 hardware activation completed without homing.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EmbodiedRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(robot_motor_data_.size() * 2U);

  for (auto & motor : robot_motor_data_)
  {
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.actual_position);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.actual_velocity);
  }

  return state_interfaces;
}

hardware_interface::return_type EmbodiedRobotSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & motor : robot_motor_data_)
  {
    const auto velocity_interface = motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

    if (
      std::find(stop_interfaces.begin(), stop_interfaces.end(), velocity_interface) !=
      stop_interfaces.end())
    {
      motor.interfaces_to_running.clear();
    }

    if (
      std::find(start_interfaces.begin(), start_interfaces.end(), velocity_interface) ==
      start_interfaces.end())
    {
      continue;
    }

    const auto velocity_it = motor.command_interface_to_operation_mode.find(velocity_interface);
    if (velocity_it == motor.command_interface_to_operation_mode.end())
    {
      RCLCPP_ERROR(
        robot_system_logger,
        "No velocity mode registered for joint '%s' while switching command interfaces.",
        motor.joint_name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (!switch_operation_mode_via_sdo(motor, velocity_it->second, "command mode switch"))
    {
      return hardware_interface::return_type::ERROR;
    }

    motor.interfaces_to_running = {velocity_interface};
    motor.interfaces_to_start.clear();
    motor.interfaces_to_stop.clear();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EmbodiedRobotSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  for (auto & motor : robot_motor_data_)
  {
    const double direction_sign = joint_direction_sign(motor.joint_name);
    motor.actual_position = direction_sign * motor.driver->get_position();
    motor.actual_velocity = direction_sign * motor.driver->get_speed();
    motor.actual_effort = 0.0;
  }

  // Poll fault state at a low rate to catch EPOS4 red-LED conditions without
  // spamming the bus every control cycle.
  fault_poll_counter_++;
  if (fault_poll_counter_ >= 25U)
  {
    fault_poll_counter_ = 0U;
    for (auto & motor : robot_motor_data_)
    {
      poll_and_log_fault_state(motor);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EmbodiedRobotSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  for (auto & motor : robot_motor_data_)
  {
    const auto velocity_interface = motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    if (
      std::find(
        motor.interfaces_to_running.begin(), motor.interfaces_to_running.end(), velocity_interface) ==
      motor.interfaces_to_running.end())
    {
      continue;
    }

    if (std::isnan(motor.target_velocity))
    {
      continue;
    }

    const double scale = velocity_scale_for_joint(motor.joint_name);
    const double direction_sign = joint_direction_sign(motor.joint_name);
    const auto scaled_target =
      static_cast<int32_t>(std::llround(motor.target_velocity * scale * direction_sign));

    ros2_canopen::COData target_velocity = {
      0x60FF, 0x00, static_cast<uint32_t>(scaled_target)};

    if (!motor.driver->tpdo_transmit(target_velocity))
    {
      RCLCPP_ERROR(
        robot_system_logger,
        "Failed to transmit target velocity for joint '%s'.",
        motor.joint_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ar_project

PLUGINLIB_EXPORT_CLASS(ar_project::EmbodiedRobotSystem, hardware_interface::SystemInterface)
