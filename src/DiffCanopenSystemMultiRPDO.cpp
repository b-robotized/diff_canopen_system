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
#include "diff_canopen_system/DiffCanopenSystemMultipRPDO.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("DiffCanopenSystemMultiRPDO");
}

namespace diff_canopen_system
{
DiffCanopenSystemMultiRPDO::DiffCanopenSystemMultiRPDO(): CanopenSystem() {};

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DiffCanopenSystemMultiRPDO::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto init_rval = CanopenSystem::on_init(info);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // Check parameters consistency for canopen joints
    if (info_.joints[i].parameters.find("node_id") != info_.joints[i].parameters.end())
    {
      if (info_.joints[i].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", info_.joints[i].name.c_str(),
            info_.joints[i].state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.joints[i].state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", info_.joints[i].name.c_str(),
            info_.joints[i].state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }
    }
  }

  return init_rval;
}


std::vector<hardware_interface::StateInterface> DiffCanopenSystemMultiRPDO::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint node_id = static_cast<uint>(std::stoi(info_.joints[i].parameters["node_id"]));

    // Mapping
    uint16_t position_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["state_interface__position__index"]));
    uint8_t position_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["state_interface__position__subindex"]));

    uint16_t velocity_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["state_interface__velocity__index"]));
    uint8_t velocity_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["state_interface__velocity__subindex"]));
    
    PDO_INDICES position_pdo_indices(position_index, position_subindex);
    PDO_INDICES velocity_pdo_indices(velocity_index, velocity_subindex);
    
    // Make pair
    NODE_PDO_INDICES position_node_pdos(node_id, position_pdo_indices);
    NODE_PDO_INDICES velocity_node_pdos(node_id, velocity_pdo_indices);

    // Intialize the value
    state_ro_.emplace(position_node_pdos, 0.0);
    state_ro_.emplace(velocity_node_pdos, 0.0);

    // state
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &state_ro_[position_node_pdos]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &state_ro_[position_node_pdos]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffCanopenSystemMultiRPDO::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces for non-can joints
      continue;
    }
  
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // Mapping - TODO(): Check interface type
    uint16_t velocity_ref_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["command_interface__velocty__index"]));
    uint8_t velocity_ref_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["command_interface__velocty__subindex"]));
    
    PDO_INDICES velocity_ref_indices(velocity_ref_index, velocity_ref_subindex);
    
    // Make pair
    NODE_PDO_INDICES velocity_ref_node_indices(node_id, velocity_ref_indices);
    velocity_command_.emplace(velocity_ref_node_indices, 0.0);

    // register the interface
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_command_[velocity_ref_node_indices]));
  }
  return command_interfaces;
}


hardware_interface::return_type DiffCanopenSystemMultiRPDO::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret_val = CanopenSystem::read(time, period);
  // Find a mapping between RPDOs and the state variables..
  // This for loop read the current value from the different joints.
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint node_id = static_cast<uint>(std::stoi(info_.joints[i].parameters["node_id"]));

    PDO_INDICES rpod_indices;
    rpod_indices.first = canopen_data_[node_id].rpdo_data.original_data.index_;
    rpod_indices.second = canopen_data_[node_id].rpdo_data.original_data.subindex_;

    NODE_PDO_INDICES node_rpdo_indices(node_id, rpod_indices);
    state_ro_[node_rpdo_indices] = canopen_data_[node_id].rpdo_data.data;
  }

  return ret_val;
}

hardware_interface::return_type DiffCanopenSystemMultiRPDO::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret_val = CanopenSystem::write(time, period);
  
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

    // TODO: Add the mapping right here
    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // Convert percents command to speed data
      it->second.tpdo_data.data = it->second.tpdo_data.data;
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
  diff_canopen_system::DiffCanopenSystemMultiRPDO, hardware_interface::SystemInterface)
