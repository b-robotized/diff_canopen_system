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
#include "diff_canopen_system/custom_mapping_canopen_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("CustomMappingCanopenSystem");
}

namespace custom_mapping_canopen_system
{
CustomMappingCanopenSystem::CustomMappingCanopenSystem() : CanopenSystem() {};

hardware_interface::CallbackReturn CustomMappingCanopenSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto ret_val = CanopenSystem::on_init(info);
  if (ret_val != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret_val;
  }

  // lambda to simple check if parameter exist
  auto check_parameter_exist = [](
    const std::unordered_map<std::string, std::string> & param_map, const std::string & param_name, const std::string & interface_name,
    const std::string & joint_name)
  {
    if (param_map.find(param_name) == param_map.end())
    {
      RCLCPP_FATAL(kLogger, "Missing '%s' parameter for interface '%s' on joint '%s'!", param_name.c_str(), interface_name.c_str(), joint_name.c_str());
      return false;
    }
    return true;
  };

  commands_.reserve(info_.joints.size());
  states_.reserve(info_.joints.size());

  // Check all CANOpen parameters to detect any errors immediately
  for (const auto & joint : info_.joints)
  {
    if (joint.parameters.find("node_id") == joint.parameters.end())
    {
      RCLCPP_FATAL(kLogger, "Missing 'node_id' parameter for joint '%s'!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    // TODO(Dr. Denis): add try catch
    node_id_t node_id = static_cast<node_id_t>(std::stoi(joint.parameters.at("node_id"), nullptr, 0));  // cast from any base

    commands_[joint.name] = std::unordered_map<std::string, InterfaceToCanOpen>();
    commands_[joint.name].reserve(joint.command_interfaces.size());
    // TODO(Dr. Denis): do the same for the `state_interfaces` but scaling is then other way around
    for (const auto & interface : joint.command_interfaces)
    {
      if (!check_parameter_exist(interface.parameters, "index", interface.name, joint.name) ||
          !check_parameter_exist(interface.parameters, "subindex", interface.name, joint.name))
      {
        return CallbackReturn::ERROR;
      }
      auto itf_to_canopen = InterfaceToCanOpen();
      itf_to_canopen.info = interface;
      itf_to_canopen.node_id = node_id;
      itf_to_canopen.data = std::make_shared<canopen_ros2_control::WORos2ControlCoData>();
      try {
        // write to "data" for the WOROS2ControlCoData
        itf_to_canopen.data->index = static_cast<uint16_t>(std::stoi(interface.parameters.at("index"), nullptr, 0));  // cast from any base
        itf_to_canopen.data->subindex = static_cast<uint8_t>(std::stoi(interface.parameters.at("subindex"), nullptr, 0));  // cast from any base
        itf_to_canopen.data->data = 0.0;
      }
      catch (const std::invalid_argument & e)
      {
        RCLCPP_FATAL(kLogger, "Invalid argument in 'index' or 'subindex' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      catch (const std::out_of_range & e)
      {
        RCLCPP_FATAL(kLogger, "Out of range in 'index' or 'subindex' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }

      // check other parameters
      {
        const auto it = std::find_if(
          interface.parameters.begin(), interface.parameters.end(),
          [](const auto & param) { return param.first == "unit"; });
        if (it != interface.parameters.end())
        {
          if (it->second == "rpm")
          {
            itf_to_canopen.scale_factor *= 60.0 / (2.0 * M_PI);
          }
        }
      }
      {
        const auto it = std::find_if(
          interface.parameters.begin(), interface.parameters.end(),
          [](const auto & param) { return param.first == "to_hw_scaling_factor"; });
        if (it != interface.parameters.end())
        {
          itf_to_canopen.scale_factor *= std::stod(it->second);
        }
      }

      commands_[joint.name].emplace(interface.name, itf_to_canopen);
    }

    states_[joint.name] = std::unordered_map<std::string, InterfaceToCanOpen>();
    states_[joint.name].reserve(joint.state_interfaces.size());
    for (const auto & interface : joint.state_interfaces)
    {

      if (!check_parameter_exist(interface.parameters, "index", interface.name, joint.name) ||
          !check_parameter_exist(interface.parameters, "subindex", interface.name, joint.name))
      {
        return CallbackReturn::ERROR;
      }
      auto itf_to_canopen = InterfaceToCanOpen();
      itf_to_canopen.info = interface;
      itf_to_canopen.node_id = node_id;
      itf_to_canopen.data = std::make_shared<canopen_ros2_control::RORos2ControlCOData>();
      try {
        itf_to_canopen.data->original_data = 
          ros2_canopen::COData{
            static_cast<uint16_t>(std::stoi(interface.parameters.at("index"), nullptr, 0)),  // cast from any base
            static_cast<uint8_t>(std::stoi(interface.parameters.at("subindex"), nullptr, 0)),  // cast from any base
            static_cast<uint32_t>(0)
          };
        }
      catch (const std::invalid_argument & e)
      {
        RCLCPP_FATAL(kLogger, "Invalid argument in 'index' or 'subindex' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      catch (const std::out_of_range & e)
      {
        RCLCPP_FATAL(kLogger, "Out of range in 'index' or 'subindex' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      
      // check other parameters
      {
        const auto it = std::find_if(
          interface.parameters.begin(), interface.parameters.end(),
          [](const auto & param) { return param.first == "unit"; });
        if (it != interface.parameters.end())
        {
          if (it->second == "rpm")
          {
            itf_to_canopen.scale_factor *= (2.0 * M_PI) / 60.0;
          }
        }
      }
      {
        const auto it = std::find_if(
          interface.parameters.begin(), interface.parameters.end(),
          [](const auto & param) { return param.first == "from_hw_scaling_factor"; });
        if (it != interface.parameters.end())
        {
          itf_to_canopen.scale_factor *= std::stod(it->second);
        }
      }

      states_[joint.name].emplace(interface.name, itf_to_canopen);
    }

    last_toggled_bit_.emplace(joint.name, false);
    controller_state_.emplace(joint.name, ControllerStates::DISABLED);
  }

  return ret_val;
}

std::vector<hardware_interface::StateInterface> CustomMappingCanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // add CANOpen interfaces
  if (info_.hardware_parameters.find("enable_canopen_interfaces") != info_.hardware_parameters.end() && 
      info_.hardware_parameters["enable_canopen_interfaces"] == "true")
  {
    state_interfaces = CanopenSystem::export_state_interfaces();
  }

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : states_[joint.name])
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, 
        interface.second.info.name, 
        &interface.second.data->data));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CustomMappingCanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // add CANOpen interfaces
  if (info_.hardware_parameters.find("enable_canopen_interfaces") != info_.hardware_parameters.end() && 
      info_.hardware_parameters["enable_canopen_interfaces"] == "true")
  {
    command_interfaces = CanopenSystem::export_command_interfaces();
  }

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : commands_[joint.name])
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, 
        interface.second.info.name, 
        &interface.second.data->data));

    }
  }
  return command_interfaces;
}


hardware_interface::return_type CustomMappingCanopenSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (info_.hardware_parameters.find("enable_canopen_interfaces") != info_.hardware_parameters.end() && 
      info_.hardware_parameters["enable_canopen_interfaces"] == "true")
  {
    auto ret_val = CanopenSystem::read(time, period);
    // if not OK then return with error
    if (ret_val != hardware_interface::return_type::OK)
    {
      RCLCPP_ERROR(kLogger, "Error has hapend in underlaying CanopenSystem::read call. See above for more details.");
      return ret_val;
    }
  }

  for (const auto & joint : info_.joints)
  {
    // TODO(Dr. Denis): Can we here avoid parsing of the node_id?
    node_id_t node_id = 0;
    if (!states_[joint.name].empty())
    {
      // TODO(Dr. Denis): Can we here avoid parsing of the node_id?
      node_id = states_[joint.name].begin()->second.node_id;
    }
    else
    {
      // no state interfaces - therefore return
      return hardware_interface::return_type::OK;
    }

    ControllerStatusByte status_byte;

    for (const auto & interface : states_[joint.name])
    {
      const auto itf_to_co = interface.second;
      // then, we are manually accessing original data "private" structure members, this is not clean
      // this could be simplified if we would use a map of `rpdo_data` in the CanopenNodeData class, where we would then "register" our data the we have initilized above - just an idea to simplify this! Nevertheless, the first thing would be to move this code to `CanopenSystem` class, as then the things get clearer and simpler
      const auto data = canopen_data_[node_id].get_rpdo_data(
        itf_to_co.data->original_data.index_, 
        itf_to_co.data->original_data.subindex_);
      itf_to_co.data->data = scale(data, itf_to_co.scale_factor);

      // TODO: add here automatic casting to the data which should be variant from ros2_control

      // BEGIN: Controller specific implementation
      if (itf_to_co.info.name == "fault")
      {
        status_byte.fault = static_cast<bool>(itf_to_co.data->data);
      }
      if (itf_to_co.info.name == "power_stage_active")
      {
        status_byte.power_stage_active = static_cast<bool>(itf_to_co.data->data);

      }

      if (itf_to_co.info.name == "toggle_bit")
      {
        const auto bit = static_cast<bool>(itf_to_co.data->data);
        if (last_toggled_bit_[joint.name] == bit)
        {
          RCLCPP_WARN(kLogger, "Toggle bit is not toggelded!");
        }
        last_toggled_bit_[joint.name] = bit;
      }
      // END: Controller specific implementation
    }

    // BEGIN: Controller State machine
    if (status_byte.fault)
    {
      controller_state_[joint.name] = ControllerStates::FAULT;
    }
    else
    {
      controller_state_[joint.name] = ControllerStates::ENABLED;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CustomMappingCanopenSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (info_.hardware_parameters.find("enable_canopen_interfaces") != info_.hardware_parameters.end() && 
      info_.hardware_parameters["enable_canopen_interfaces"] == "true")
  {
    auto ret_val = CanopenSystem::write(time, period);
    // if not OK then return with error
    if (ret_val != hardware_interface::return_type::OK)
    {
      RCLCPP_ERROR(kLogger, "Error has hapend in underlaying CanopenSystem::write call. See above for more details.");
      return ret_val;
    }
  }

  auto drivers = device_container_->get_registered_drivers();

  for (const auto & joint : info_.joints)
  {
    node_id_t node_id = 0;
    if (!commands_[joint.name].empty())
    {
      node_id = commands_[joint.name].begin()->second.node_id;
    }
    else
    {
      // no command interfaces - therefore return
      return hardware_interface::return_type::OK;
    }

    // BEGIN: Custom startup sequence for the motor controller - this should be part of a controller or custom driver


    // END: Custom startup sequence for the motor controller - this should be part of a controller or custom driver

    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[node_id]);
    for (const auto & interface : commands_[joint.name])
    {
      const auto itf_to_co = interface.second;

      const auto data_to_transmit = std::static_pointer_cast<canopen_ros2_control::WORos2ControlCoData>(itf_to_co.data);

      itf_to_co.data->data = scale(itf_to_co.data->data, itf_to_co.scale_factor);
      // RCLCPP_INFO(kLogger, "NodeID: 0x%X; Index: 0x%X; Original Data: %u; Itf data: %f",
      //             itf_to_co.node_id, itf_to_co.data->original_data.index_, itf_to_co.data->original_data.data_,
      //             itf_to_co.data->data);
      data_to_transmit->prepare_data();
      proxy_driver->tpdo_transmit(data_to_transmit->original_data);
    }
  }
  return hardware_interface::return_type::OK;
}

uint32_t CustomMappingCanopenSystem::scale(const double data, const double scale_factor)
{
  return static_cast<uint32_t>(data * scale_factor);
}

}  // namespace custom_mapping_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  custom_mapping_canopen_system::CustomMappingCanopenSystem, hardware_interface::SystemInterface)
