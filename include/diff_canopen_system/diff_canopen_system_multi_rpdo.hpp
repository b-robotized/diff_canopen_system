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
};

enum StateInterfaces
{
  VELOCITY,
  POSITION,
};

struct JointState {
  // Read only
  double position;
  double velocity;

  // Write only
  double velocity_command;
};

class DiffCanopenSystemMultiRPDO : public canopen_ros2_control::CanopenSystem
{
public:
  DiffCanopenSystemMultiRPDO();
  ~DiffCanopenSystemMultiRPDO() = default;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // void initDeviceContainer() override;

private:
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

  // using RPDO_INDICES_MAPPING = std::unordered_map<PDO_INDICES, StateInterfaces, pair_hash>;
  // using TPDO_INDICES_MAPPING = std::unordered_map<PDO_INDICES, CommandInterfaces, pair_hash>;
  // std::unordered_map<unit, RPDO_INDICES_MAPPING> rpdo_mapping_;
  // std::unordered_map<unit, TPDO_INDICES_MAPPING> tpdo_mapping_;

  // States - Read only
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> state_ro_;

  // std::unordered_map<unit, double> position_ro_;
  // std::unordered_map<unit, double> velocity_ro_;

  // Command
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> velocity_command_; 
};

}  // namespace diff_canopen_system

#endif  // DIFF_CANOPEN_SYSTEM__DIFFCANOPENSYSTEM_HPP_
