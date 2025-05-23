// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIFF_CANOPEN_SYSTEM__DIFFCANOPENSYSTEM_HPP_
#define DIFF_CANOPEN_SYSTEM__DIFFCANOPENSYSTEM_HPP_

#include <string>
#include <vector>

#include "diff_canopen_system/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "canopen_ros2_control/canopen_system.hpp"

namespace diff_canopen_system
{

enum CommandInterfaces
{
  VELOCITY_REFERENCE,
  NMT_RESET,
  NMT_RESET_FBK,
  NMT_START,
  NMT_START_FBK,
};

enum StateInterfaces
{
  VELOCITY_REF,
  VELOCITY_FEEDBACK,
  MOTOR_TEMPERATURE,
  MOTOR_POWER,
  BATTERY_STATE,
  ERROR_STATUS,
  NMT_STATE,
};

struct WheelState {
  // Read only
  double velocity_reference;
  double velocity_feedback;
  double motor_temperature;
  double motor_power;
  double motor_battery_state;
  double error_status; 

  // Write only
  double velocity_command;
};

class DiffCanopenSystem : public canopen_ros2_control::CanopenSystem
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  DiffCanopenSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~DiffCanopenSystem() = default;
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration & period) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration & period) override;

protected:
  // void initDeviceContainer() override;

private:
  // State converter
  uint32_t convert_percentage_to_speed_value(const double);
  double convert_rpm_to_rads(const uint32_t);

  // States
  std::unordered_map<uint, WheelState> wheel_states_;
};

}  // namespace diff_canopen_system

#endif  // DIFF_CANOPEN_SYSTEM__DIFFCANOPENSYSTEM_HPP_
