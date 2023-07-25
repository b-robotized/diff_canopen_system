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
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DiffCanopenSystemMultiRPDO::on_init(
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
      if (!check_parameter_exist(info_.joints[i].parameters, "veloctiy_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "velocity_subindex", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "position_index", info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, "position_subindex", info_.joints[i].name))
      {
        init_rval = CallbackReturn::ERROR;
      }
    }
  }

  return init_rval;
}


std::vector<hardware_interface::StateInterface> DiffCanopenSystemMultiRPDO::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();
  position_ro_.resize(info_.joints.size());
  velocity_ro_.resize(info_.joints.size());
  rpdo_mapping_.resize(info_.joints.size());

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
    
    // Mapping
    const uint16_t position_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["position_index"]));
    const uint8_t position_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["position_subindex"]));

    const uint16_t velocity_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["velocity_index"]));
    const uint8_t velocity_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["velocity_subindex"]));
    
    auto position_pdo_indices = std::make_pair<uint16_t, uint8_t>(position_index, position_subindex);
    auto velocity_pdo_indices = std::make_pair<uint16_t, uint8_t>(velocity_index, velocity_subindex);
    
    PDO_INDICES pdo_mapping;
    pdo_mapping.emplace(position_pdo_indices, StateInterfaces::POSITION);
    pdo_mapping.emplace(velocity_pdo_indices, StateInterfaces::VELOCITY);
    rpdo_mapping_.push_back[pdo_mapping];
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffCanopenSystemMultiRPDO::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  velocity_command_.resize(info_.joints.size());
  tpdo_mapping_.resize(info_.joints.size());

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

     // Mapping
    const uint16_t velocity_ref_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters["velocity_ref_index"]));
    const uint8_t velocity_ref_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters["velocity_ref_subindex"]));
    
    auto velocity_ref_indices = std::make_pair<uint16_t, uint8_t>(
      velocity_ref_index, velocity_ref_subindex);
    
    PDO_INDICES pdo_mapping;
    pdo_mapping.emplace(velocity_ref_indices, CommandInterfaces::VELOCITY_REFERENCE);
    tpdo_mapping_.push_back[pdo_mapping];

  }
  return command_interfaces;
}


hardware_interface::return_type DiffCanopenSystemMultiRPDO::read()
{
  // Find a mapping between RPDOs and the state variables..
  // This for loop read the current value from the different joints.
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    // TODO(): Do the mapping right here
    
    PDO_INDICES rpod_indices;
    rpod_indices.first = it->second.rpdo_data.original_data.index_;
    rpod_indices.second = it->second.rpdo_data.original_data.subindex_;

    auto interface = rpdo_mapping_[it->first][rpod_indices];

    switch (interface) {
      case StateInterfaces::VELOCITY: 
        velocity_ro_[it->first] = it->second.rpdo_data.data;
        break;

      case StateInterfaces::POSITION:    
        position_ro_[it->first] = it->second.rpdo_data.data;
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffCanopenSystemMultiRPDO::write()
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
