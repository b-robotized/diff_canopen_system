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

#include <limits>
#include <vector>
#include "diff_canopen_system/diffCanopenSystemMultiRPDO.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("diffCanopenSystemMultiRPDO");
}

namespace diff_canopen_system
{
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn diffCanopenSystemMultiRPDO::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto init_rval = CanopenSystem::on_init(info);

  auto check_parameter_exist = [](
    const std::unordered_map<std::string, std::string> & param_map, const std::string & param_name,
    const std::string & joint_name)
  {
    if (param_map.find(param_name) == param_map.end())
    {
      RCLCPP_FATAL(kLogger, "Missing '%s' parameter for joint '%s'!", param_name.c_str(), joint_name.c_str());
      return false;
    }
    return true;
  };

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // Check parameters consistency for canopen joints
    if (info_.joints[i].parameters.find("node_id") != info_.joints[i].parameters.end())
    {
      // TODO: We should modify this part
      if (!check_parameter_exist(info_.joints[i].parameters, "velocity_cmd_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "velocity_ref_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "velocity_fbk_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "motor_temperature_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "motor_power_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "battery_state_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "error_status_index", info_.joints[i].name))
      {
        init_rval = CallbackReturn::ERROR;
      }
    }
  }

  return init_rval;
}


std::vector<hardware_interface::StateInterface> diffCanopenSystemMultiRPDO::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();
  position_ro_.resize(info_.joints.size());
  velocity_ro_.resize(info_.joints.size());

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // state
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      position_ro_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      velocity_ro_[i]));
  }
  // Send request

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> diffCanopenSystemMultiRPDO::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.resize(info_.joints.size());
  velocity_command_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces for non-can joints
      continue;
    }
  
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));
  
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      velocity_command_[i]));

  }
  return command_interfaces;
}


hardware_interface::return_type diffCanopenSystemMultiRPDO::read()
{
  // Find a mapping between RPDOs and the state variables..
  // This for loop read the current value from the different joints.
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    // TODO(): Do the mapping right here
    bool velocity_rpdo = false;
    bool position_rpdo = false;

    if (velocity_rpdo) {
      velocity_ro_[i] = static_cast<double>(it->second.rpdo_data.data);
    }

    if (position_rpdo) {
      position_ro_[i] = static_cast<double>(it->second.rpdo_data.data);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffCanopenSystemMultiRPDO::write()
{
  auto drivers = device_container_->get_registered_drivers();

  // TODO: delete the message that we do not need.

  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

    // reset node nmt
    if (it->second.nmt_state.reset_command())
    {
      it->second.nmt_state.reset_fbk = static_cast<double>(proxy_driver->reset_node_nmt_command());
    }

    // start nmt
    if (it->second.nmt_state.start_command())
    {
      it->second.nmt_state.start_fbk = static_cast<double>(proxy_driver->start_node_nmt_command());
    }

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // Convert percents command to speed data
      it->second.tpdo_data.data = convert_percentage_to_speed_value(it->second.tpdo_data.data);
      it->second.tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      // Debug Message
      RCLCPP_INFO(kLogger, "This is a debug message in HW-write().....");
      RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
      it->first,
      it->second.tpdo_data.original_data.index_,
      it->second.tpdo_data.original_data.subindex_,
      it->second.tpdo_data.original_data.data_);
      RCLCPP_INFO(kLogger, "--- END of the debug message in HW-write()");
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diff_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_canopen_system::diffCanopenSystemMultiRPDO, hardware_interface::SystemInterface)
