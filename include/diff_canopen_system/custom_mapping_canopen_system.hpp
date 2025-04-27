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

template<typename Target, typename Source>
Target caster(Source src)
{
    return static_cast<Target>(src);
}

std::function<int16_t(double)> int16_t_caster = caster<int16_t, double>;

using node_id_t = uint8_t;

struct InterfaceToCanOpen
{
  hardware_interface::InterfaceInfo info;
  node_id_t node_id;
  std::shared_ptr<canopen_ros2_control::Ros2ControlCOData> data;
  double scale_factor = 1.0;
};

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

protected:
  // void initDeviceContainer() override;

private:
  // States - This is a container to store states in a double form
  std::unordered_map<uint, WheelState> wheel_states_;

  // This make the std::pair hashable
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& pair) const {
        std::size_t h1 = std::hash<T1>{}(pair.first);
        std::size_t h2 = 0;

        // check if T2 is a pair, if so recursively compute hash
        if constexpr (std::is_same<T2, std::pair<uint16_t, uint8_t>>::value) {
            h2 = pair_hash{}(pair.second);
        } else {
            h2 = std::hash<T2>{}(pair.second);
        }

        return h1 ^ h2;
    }
  };

  // PDO_Interfaces_Mapping
  using PDO_INDICES = std::pair<uint16_t, uint8_t>; // Index, Subindex
  using NODE_PDO_INDICES = std::pair<uint, PDO_INDICES>; // Node_id, PDO_INDICES

  // States - Read only
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> state_ro_;

  // Command
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> velocity_command_; 

  // State converter
  typedef double (*FunctionType)(double);
  std::unordered_map<NODE_PDO_INDICES, FunctionType, pair_hash> state_converter_;

  // TODO(Dr.Denis): We might here need to update this the API changed compatibility
  // For now we develop against deprecated API!!
  std::unordered_map<std::string, std::vector<InterfaceToCanOpen>> states_;
  std::unordered_map<std::string, std::vector<InterfaceToCanOpen>> commands_;

  uint32_t scale(const double data, const double scale_factor);

  static double convert_to_position(double rpdo_data);
  static double convert_to_temperature(double rpdo_data);
  static double convert_to_veloctiy(double rpdo_data);
  static double convert_to_RPM(double rpdo_data);
  static double convert_to_switch_voltage(double rpdo_data);

  double convert_rpm_to_rads(const uint32_t rpm);
  double convert_rads_to_rpm(const double rads);
  double convert_rpm_to_percentage(const double rpm);
  uint32_t convert_percentage_to_speed_value(const double percentage);

  std::vector<PDO_INDICES> state_pdo_indices_;

  bool enable_write_ = false;
};

}  // namespace custom_mapping_canopen_system

#endif  // DIFF_CANOPEN_SYSTEM__CUSTOM_MAPPING_CANOPEN_SYSTEM_HPP_
