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
      itf_to_canopen.data->set_co_data_type(interface.data_type);
      // BEGIN: remove when getting data_type is fixed
      try
      {
        itf_to_canopen.data->set_co_data_type(interface.parameters.at("data_type"));
      }
      catch (const std::out_of_range & e)
      {} // we just ignore this
      // END: remove when getting data_type is fixed
      try
      {
        // write to "data" for the WOROS2ControlCoData
        itf_to_canopen.data->index = static_cast<uint16_t>(std::stoi(interface.parameters.at("index"), nullptr, 0));  // cast from any base
        itf_to_canopen.data->subindex = static_cast<uint8_t>(std::stoi(interface.parameters.at("subindex"), nullptr, 0));  // cast from any base
        itf_to_canopen.data->data = interface.initial_value.empty() ? 0.0 : std::stod(interface.initial_value);
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

      if (!interface.min.empty())
      {
        try {
          itf_to_canopen.min = std::stod(interface.min);
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_FATAL(kLogger, "Invalid argument in 'min' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
          return CallbackReturn::ERROR;
        }
      }
      if (!interface.max.empty())
      {
        try {
          itf_to_canopen.max = std::stod(interface.max);
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_FATAL(kLogger, "Invalid argument in 'max' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
          return CallbackReturn::ERROR;
        }
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
          RCLCPP_INFO(kLogger, "Got factor %f TO hw for interface '%s' on joint '%s'.", itf_to_canopen.scale_factor, interface.name.c_str(), joint.name.c_str());
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
      itf_to_canopen.data->set_co_data_type(interface.data_type);  // TODO(Dr. Denis): investigate why this attribute is not set. Very confusing!!!
      // BEGIN: remove when getting data_type is fixed
      try
      {
        itf_to_canopen.data->set_co_data_type(interface.parameters.at("data_type"));
      }
      catch (const std::out_of_range & e)
      {} // we just ignore this
      // END: remove when getting data_type is fixed
      try 
      {
        itf_to_canopen.data->original_data = 
          ros2_canopen::COData{
            static_cast<uint16_t>(std::stoi(interface.parameters.at("index"), nullptr, 0)),  // cast from any base
            static_cast<uint8_t>(std::stoi(interface.parameters.at("subindex"), nullptr, 0)),  // cast from any base
            static_cast<uint32_t>(0)
          };
        itf_to_canopen.data->data = interface.initial_value.empty() ? 0.0 : std::stod(interface.initial_value);
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

      RCLCPP_INFO(kLogger, "Added interface: NodeID: 0x%X; Index: 0x%X; Subindex: 0x%X; Type: %s; Raw data: %u; Data (%f). Target type: %s.",
                    itf_to_canopen.node_id,
                    itf_to_canopen.data->original_data.index_,
                    itf_to_canopen.data->original_data.subindex_,
                    itf_to_canopen.data->co_type.c_str(),
                    itf_to_canopen.data->original_data.data_,
                    itf_to_canopen.data->data,
                    interface.data_type.c_str());

      if (!interface.min.empty())
      {
        try {
          itf_to_canopen.min = std::stod(interface.min);
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_FATAL(kLogger, "Invalid argument in 'min' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
          return CallbackReturn::ERROR;
        }
      }
      if (!interface.max.empty())
      {
        try {
          itf_to_canopen.max = std::stod(interface.max);
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_FATAL(kLogger, "Invalid argument in 'max' for interface '%s' on joint '%s': %s", interface.name.c_str(), joint.name.c_str(), e.what());
          return CallbackReturn::ERROR;
        }
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
          RCLCPP_INFO(kLogger, "Got factor %f FROM hw for interface '%s' on joint '%s'.", itf_to_canopen.scale_factor, interface.name.c_str(), joint.name.c_str());
        }
      }

      states_[joint.name].emplace(interface.name, itf_to_canopen);
    }

    last_toggled_bit_.emplace(joint.name, false);
    controller_state_.emplace(joint.name, ControllerStates::POWER_OFF);
    init_sequence_.emplace(joint.name, InitSequence());
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

  if (canopen_data_.size() != (info_.joints.size() + 1))
  {
    RCLCPP_WARN(kLogger, "Size of canopen_data_ (%zu) is smaller then number of joints (%zu) + 1 . Some slaves are maybe not initialized!", canopen_data_.size(), info_.joints.size());
  }

  for (const auto & joint : info_.joints)
  {
    node_id_t node_id = 0;
    if (!states_.at(joint.name).empty())
    {
      node_id = states_.at(joint.name).begin()->second.node_id;
    }
    else
    {
      // no state interfaces - therefore return
      return hardware_interface::return_type::OK;
    }

    for (const auto & interface : states_.at(joint.name))
    {
      const auto itf_to_co = interface.second;
      // this could be simplified if we would use a map of `rpdo_data` in the CanopenNodeData class, where we would then "register" our data the we have initilized above - just an idea to simplify this! Nevertheless, the first thing would be to move this code to `CanopenSystem` class, as then the things get clearer and simpler
      itf_to_co.data->original_data.data_ = canopen_data_.at(node_id).get_rpdo_raw_data(
      itf_to_co.data->original_data.index_, 
      itf_to_co.data->original_data.subindex_);
      const auto dat = std::static_pointer_cast<canopen_ros2_control::RORos2ControlCOData>(itf_to_co.data);
      dat->prepare_data();

      if (itf_to_co.data->data < itf_to_co.min || itf_to_co.data->data > itf_to_co.max)
      {
        RCLCPP_WARN(kLogger, "NodeID: 0x%X; Index: 0x%X; Subindex: 0x%X; Type: %s; Raw data: %u; Data (%f) is out of bounds [%f, %f] - clamping! This might be because of a wrong type conversion! Please check the type of the data in the CANopen dictionary!",
                    node_id,
                    itf_to_co.data->original_data.index_,
                    itf_to_co.data->original_data.subindex_,
                    itf_to_co.data->co_type.c_str(),
                    itf_to_co.data->original_data.data_,
                    itf_to_co.data->data,
                    itf_to_co.min,
                    itf_to_co.max);
      }
      // clamping is done on the data before scaling
      itf_to_co.data->data = std::clamp(itf_to_co.data->data, itf_to_co.min, itf_to_co.max);
      itf_to_co.data->data = scale(itf_to_co.data->data, itf_to_co.scale_factor);
    }

    // BEGIN: Controller specific implementation of state machine
    // check if data received at least once so we have actually something to parse
    const auto bit = static_cast<bool>(states_.at(joint.name).at("toggle_bit").data->data);
    if (last_toggled_bit_.at(joint.name) != bit)  // got new data when bit is toggeled
    {
      if (static_cast<bool>(states_.at(joint.name).at("fault").data->data))
      {
        if (controller_state_.at(joint.name) != ControllerStates::FAULT)
        {
          RCLCPP_WARN(kLogger, "Setting '%s' in FAULT state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::FAULT;
      }
      else if (static_cast<bool>(states_.at(joint.name).at("safe_stop_active").data->data))
      {
        if (controller_state_.at(joint.name) != ControllerStates::SAFE_STOP)
        {
          RCLCPP_WARN(kLogger, "Setting '%s' in SAFE_STOP state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::SAFE_STOP;
      }
      else if (static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data) && 
               static_cast<bool>(states_.at(joint.name).at("main_contactor_status").data->data))
      {
        if (controller_state_.at(joint.name) != ControllerStates::POWER_ON_CONTACTOR)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_ON_CONTACTOR state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_ON_CONTACTOR;
      }
      else if (!static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data) && 
               static_cast<bool>(states_.at(joint.name).at("main_contactor_status").data->data))
      {
        // it seems that the gives bit sequence: "2" --> "1" - this is build with below states
        // (1) "Output on" on Main contactor status
        // (0) "Power stage disabled" on Power stage active
        // and then resets it to:
        // (0) "Output off" on Main contactor status
        // (1) "Power stage active" on Power stage active
        if (controller_state_.at(joint.name) != ControllerStates::POWER_OFF_CONTACTOR)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_OFF_CONTACTOR state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_OFF_CONTACTOR;
      }
      else if (static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data) && 
               !static_cast<bool>(states_.at(joint.name).at("main_contactor_status").data->data) &&
               (controller_state_.at(joint.name) == ControllerStates::POWER_OFF_CONTACTOR || 
                controller_state_.at(joint.name) == ControllerStates::FAULT))
      {
        if (controller_state_.at(joint.name) != ControllerStates::POWER_ON_CONTACTOR)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_ON_CONTACTOR state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_ON_CONTACTOR;
      }
      else if (static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data) && 
               !static_cast<bool>(states_.at(joint.name).at("main_contactor_status").data->data) &&
               (controller_state_.at(joint.name) == ControllerStates::POWER_ON_CONTACTOR || 
                controller_state_.at(joint.name) == ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE))  // keep it in running state
      {
        if (controller_state_.at(joint.name) != ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_ON_CONTACTOR_BREAK_RELEASE state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE;
      }
      else if (static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data) && 
               !static_cast<bool>(states_.at(joint.name).at("main_contactor_status").data->data) &&
               (controller_state_.at(joint.name) != ControllerStates::POWER_OFF_CONTACTOR || 
                controller_state_.at(joint.name) != ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE))
      {
        if (controller_state_.at(joint.name) != ControllerStates::POWER_ON)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_ON state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_ON;
      }
      else if (!static_cast<bool>(states_.at(joint.name).at("power_stage_active").data->data))
      {
        if (controller_state_.at(joint.name) != ControllerStates::POWER_OFF)
        {
          RCLCPP_INFO(kLogger, "Setting '%s' in POWER_OFF state.", joint.name.c_str());
        }
        controller_state_.at(joint.name) = ControllerStates::POWER_OFF;
      }
    }
    else
    {
      RCLCPP_WARN(kLogger, "Controller '%s' (0x%X): no new data (bit is not toggeled).", joint.name.c_str(), node_id);
    }
    last_toggled_bit_.at(joint.name) = bit;
    // END: Controller specific implementation    
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
    if (!commands_.at(joint.name).empty())
    {
      node_id = commands_.at(joint.name).begin()->second.node_id;
    }
    else
    {
      // no command interfaces - therefore return
      return hardware_interface::return_type::OK;
    }

    const auto itf_to_co_map = commands_.at(joint.name);

    // BEGIN: Custom startup sequence for the motor controller - this should be part of a controller or custom driver
    switch (controller_state_.at(joint.name))
    {
      case ControllerStates::FAULT:
      {
        RCLCPP_DEBUG(kLogger, "Controller '%s' is in FAULT state.", joint.name.c_str());
        itf_to_co_map.at("velocity").data->data = 0.0;  // reset velocity

        // fault is reset on rising edge, i.e. we should toggle the flag
        itf_to_co_map.at("reset_fault").data->data = !(static_cast<bool>(itf_to_co_map.at("reset_fault").data->data));
        break;
      }

      case ControllerStates::SAFE_STOP:
      {
        RCLCPP_DEBUG(kLogger, "Controller '%s' is in SAFE_STOP state.", joint.name.c_str());
        itf_to_co_map.at("break_release").data->data = false;  // activate the break
        itf_to_co_map.at("reset_fault").data->data = false;        
        // itf_to_co_map.at("velocity").data->data = 0.0; // reset velocity
        break;
      }

      case ControllerStates::POWER_OFF:
      {
        if (init_sequence_.at(joint.name).num_before_power_on < 3)
        {
          RCLCPP_DEBUG(kLogger, "Controller '%s' is in POWER_OFF state. Iteration %zu. Attempting to power on.", joint.name.c_str(), init_sequence_.at(joint.name).num_before_power_on);
          itf_to_co_map.at("drive_enable").data->data = true;  // enable drive
          itf_to_co_map.at("main_contactor").data->data = false;  // enable main contactor as there is no feedback
          itf_to_co_map.at("break_release").data->data = false;  // release break
          itf_to_co_map.at("reset_fault").data->data = false;
          itf_to_co_map.at("velocity").data->data = 0.0; // reset velocity
          ++init_sequence_.at(joint.name).num_before_power_on;
        }
        else
        {
          init_sequence_.at(joint.name).num_before_power_on = 0;
          // manually switch state for the init as there is no feedback from the controllers
          if (controller_state_.at(joint.name) != ControllerStates::POWER_ON)
          {
            RCLCPP_INFO(kLogger, "Setting '%s' in POWER_ON state from write.", joint.name.c_str());
          }
          controller_state_.at(joint.name) = ControllerStates::POWER_ON;
        }
        break;
      }

      case ControllerStates::POWER_OFF_CONTACTOR:
      {
        RCLCPP_DEBUG(kLogger, "Controller '%s' is in POWER_OFF_CONTACTOR state. Activating power stage.", joint.name.c_str());
        itf_to_co_map.at("drive_enable").data->data = true;  // enable drive
        itf_to_co_map.at("main_contactor").data->data = true;  // keep contactor on
        itf_to_co_map.at("reset_fault").data->data = false;
        itf_to_co_map.at("velocity").data->data = 0.0; // reset velocity
        break;
      }

      case ControllerStates::POWER_ON:
      {
        if (init_sequence_.at(joint.name).num_before_power_on_contactor < 7)
        {
          RCLCPP_INFO(kLogger, "Controller '%s' is in POWER_ON state. Iteration %zu. Activating contactor.", joint.name.c_str(), init_sequence_.at(joint.name).num_before_power_on_contactor);
          itf_to_co_map.at("drive_enable").data->data = true;  // enable drive
          itf_to_co_map.at("main_contactor").data->data = true;  // enable main contactor
          itf_to_co_map.at("break_release").data->data = false;  // release break
          itf_to_co_map.at("reset_fault").data->data = false;
          itf_to_co_map.at("velocity").data->data = 0.0; // reset velocity
          ++init_sequence_.at(joint.name).num_before_power_on_contactor;
        }
        else if (init_sequence_.at(joint.name).num_before_power_on_contactor == 7)
        {
          // set here actuall the fault state to reset the fault
          if (controller_state_.at(joint.name) != ControllerStates::FAULT)
          {
            RCLCPP_INFO(kLogger, "Setting '%s' in FAULT state from write.", joint.name.c_str());
          }
          controller_state_.at(joint.name) = ControllerStates::FAULT;
        }
        // From now on the controller should start answering
        break;
      }

      case ControllerStates::POWER_ON_CONTACTOR:
      {
        RCLCPP_INFO(kLogger, "Controller '%s' is in POWER_ON_CONTACTOR state. Preparing for operation.", joint.name.c_str());
        itf_to_co_map.at("drive_enable").data->data = true;  // enable drive
        itf_to_co_map.at("main_contactor").data->data = true;  // keep contactor on
        itf_to_co_map.at("break_release").data->data = true;  // release break
        itf_to_co_map.at("reset_fault").data->data = false;
        itf_to_co_map.at("velocity").data->data = 0.0; // reset velocity
        // reset before contactor counter
        init_sequence_.at(joint.name).num_before_power_on_contactor = 0;
        break;
      }

      case ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE:
      {
        RCLCPP_DEBUG(kLogger, "Controller '%s' is in POWER_ON_CONTACTOR_BREAK_RELEASE state. Running normaly.", joint.name.c_str());
        itf_to_co_map.at("drive_enable").data->data = true;  // enable drive
        itf_to_co_map.at("main_contactor").data->data = true;  // keep contactor on
        itf_to_co_map.at("break_release").data->data = true;  // release break
        itf_to_co_map.at("reset_fault").data->data = false;
        break;
      }

      default:
      {
        RCLCPP_ERROR(kLogger, "Unknown state for controller '%s'.", joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
    };
    // END: Custom startup sequence for the motor controller - this should be part of a controller or custom driver

    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers.at(node_id));
    for (const auto & [itf_name, itf_to_co] : commands_.at(joint.name))
    {
      if (itf_to_co.data->data == std::numeric_limits<double>::quiet_NaN())
      {
        RCLCPP_WARN(kLogger, "NodeID: 0x%X; Index: 0x%X; Subindex: 0x%X; Data is NaN!",
          node_id,
          itf_to_co.data->original_data.index_,
          itf_to_co.data->original_data.subindex_);
        // skip if data is not yet set
        continue;
      }
      
      const auto data_to_transmit = std::static_pointer_cast<canopen_ros2_control::WORos2ControlCoData>(itf_to_co.data);
      itf_to_co.data->data = scale(itf_to_co.data->data, itf_to_co.scale_factor);
      itf_to_co.data->data = std::clamp(itf_to_co.data->data, itf_to_co.min, itf_to_co.max);

      data_to_transmit->prepare_data();
      // RCLCPP_INFO(kLogger, "Controller '%s', NodeID: 0x%X; Index: 0x%X; Original Data: %u; Itf data: %f; Interface '%s'",
      //           joint.name.c_str(), itf_to_co.node_id, itf_to_co.data->original_data.index_, itf_to_co.data->original_data.data_,
      //           itf_to_co.data->data, itf_name.c_str());
      proxy_driver->tpdo_transmit(data_to_transmit->original_data);
    }
  }
  return hardware_interface::return_type::OK;
}

double CustomMappingCanopenSystem::scale(const double data, const double scale_factor)
{
  return data * scale_factor;
}

}  // namespace custom_mapping_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  custom_mapping_canopen_system::CustomMappingCanopenSystem, hardware_interface::SystemInterface)
