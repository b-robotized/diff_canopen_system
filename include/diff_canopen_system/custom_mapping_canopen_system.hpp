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

#ifndef DIFF_CANOPEN_SYSTEM__CUSTOM_MAPPING_CANOPEN_SYSTEM_HPP_
#define DIFF_CANOPEN_SYSTEM__CUSTOM_MAPPING_CANOPEN_SYSTEM_HPP_

#include <limits>
#include <string>
#include <vector>

#include "canopen_ros2_control/canopen_system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace custom_mapping_canopen_system
{

using node_id_t = uint8_t;

struct InterfaceToCanOpen
{
  hardware_interface::InterfaceInfo info;
  node_id_t node_id;
  std::shared_ptr<canopen_ros2_control::Ros2ControlCOData> data;
  double scale_factor = 1.0;
  double min = std::numeric_limits<double>::lowest();
  double max = std::numeric_limits<double>::max();
};

enum ControllerStates {
  POWER_OFF = 0,
  POWER_ON,
  POWER_OFF_CONTACTOR,
  POWER_ON_CONTACTOR,
  POWER_ON_CONTACTOR_BREAK_RELEASE,
  FAULT,
  SAFE_STOP,
};

struct InitSequence 
{
  size_t num_before_power_on = 0;
  size_t num_before_power_on_contactor = 0;
  bool should_expect_answer = false;
};

class CustomMappingCanopenSystem : public canopen_ros2_control::CanopenSystem
{
public:
  CustomMappingCanopenSystem();
  ~CustomMappingCanopenSystem() = default;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // TODO(Dr.Denis): We might here need to update this the API changed compatibility
  // For now we develop against deprecated API!!
  std::unordered_map<std::string, std::unordered_map<std::string, InterfaceToCanOpen>> states_;
  std::unordered_map<std::string, std::unordered_map<std::string, InterfaceToCanOpen>> commands_;

  double scale(const double data, const double scale_factor);

  // BEGIN: Controller specific
  std::unordered_map<std::string, bool> last_toggled_bit_;
  std::unordered_map<std::string, ControllerStates> controller_state_;
  std::unordered_map<std::string, InitSequence> init_sequence_;
  // END: Controller specific
};

}  // namespace custom_mapping_canopen_system

#endif  // DIFF_CANOPEN_SYSTEM__CUSTOM_MAPPING_CANOPEN_SYSTEM_HPP_
