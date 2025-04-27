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

    commands_[joint.name] = std::vector<InterfaceToCanOpen>();
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
      // TODO(Dr. Denis): maybe no need to iterate, but check them directly with std::find
      for (const auto & param : interface.parameters)
      {
        // TODO(Dr. Denis): we define some standad units as we can then automatically convert between them
        if (param.first == "unit")
        {
          if (param.second == "rpm")
          {
            itf_to_canopen.scale_factor = 60.0 / (2.0 * M_PI);
          }
          else if (param.second == "scale")
          {
            // TODO(Dr. Denis): add here implementation to get value in as "scale" and here has to exist - do this in lamda function and add reading of scale also with "rpm" - as we might also have 0.1 rpm as the Unit in the datasheet - in ros2_control we always use SI units!
            itf_to_canopen.scale_factor = 1.0;
          }
          else
          {
            RCLCPP_FATAL(kLogger, "Unit '%s' for joint '%s' unknown!", param.second.c_str(), joint.name.c_str());
            return CallbackReturn::ERROR;
          }
        }
      }
      commands_[joint.name].push_back(itf_to_canopen);
    }


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
      // TODO(Dr. Denis): maybe no need to iterate, but check them directly with std::find
      for (const auto & param : interface.parameters)
      {
        // TODO(Dr. Denis): we define some standad units as we can then automatically convert between them
        if (param.first == "unit")
        {
          if (param.second == "rpm")
          {
            itf_to_canopen.scale_factor = 60.0 / (2.0 * M_PI);
          }
          else if (param.second == "scale")
          {
            // TODO(Dr. Denis): add here implementation to get value in as "scale" and here has to exist - do this in lamda function and add reading of scale also with "rpm" - as we might also have 0.1 rpm as the Unit in the datasheet - in ros2_control we always use SI units!
            itf_to_canopen.scale_factor = 1.0;
          }
          else
          {
            RCLCPP_FATAL(kLogger, "Unit '%s' for joint '%s' unknown!", param.second.c_str(), joint.name.c_str());
            return CallbackReturn::ERROR;
          }
        }
      }
      states_[joint.name].push_back(itf_to_canopen);
    }
  }

  // TODO: remove, this is debug to see itnerfaces to canopen in case 
  RCLCPP_INFO(kLogger, "COMMAND InterfaceToCanopen");
  for (const auto& pair : commands_) {
      RCLCPP_INFO(kLogger, "\t%s", pair.first.c_str());
      for (auto& interface : pair.second) {
          RCLCPP_INFO(kLogger, "\t\t%s", interface.info.name.c_str());
      }
  }

  RCLCPP_INFO(kLogger, "STATE InterfaceToCanopen");
  for (const auto& pair : states_) {
      RCLCPP_INFO(kLogger, "\t%s", pair.first.c_str());
      for (auto& interface : pair.second) {
          RCLCPP_INFO(kLogger, "\t\t%s", interface.info.name.c_str());
      }
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
        interface.info.name, 
        &interface.data->data));
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
        interface.info.name, 
        &interface.data->data));

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
    if ( !states_[joint.name].empty())
    {
      // TODO(Dr. Denis): Can we here avoid parsing of the node_id?
      node_id = commands_[joint.name][0].node_id;
    }

    for (const auto & interface : states_[joint.name])
    {
      //TODO: "ReadOnly ROs2ControlCoData" has set_data() function? how if it is read only?
      // then, we are manually accessing original data "private" structure members, this is not clean
      interface.data->data = canopen_data_[node_id].get_rpdo_data(
        interface.data->original_data.index_, 
        interface.data->original_data.subindex_);
    //TODO: apply scale factor: CAN --> ros2_control
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
    if ( !commands_[joint.name].empty())
    {
      // TODO(Dr. Denis): Can we here avoid parsing of the node_id?
      node_id = commands_[joint.name][0].node_id;
    }

    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[node_id]);
    for (const auto & interface : commands_[joint.name])
    {
      auto data_to_transmit = std::static_pointer_cast<canopen_ros2_control::WORos2ControlCoData>(interface.data);

      //TODO: apply scale factor: ros2_control --> CAN
      // interface.data->data = scale(interface.data->data, interface.scale_factor);
      data_to_transmit->prepare_data();
      proxy_driver->tpdo_transmit(data_to_transmit->original_data);

      /* DEBUG 
      RCLCPP_ERROR(kLogger, "Writing: NodeID: 0x%X  |  index: 0x%X  |  subindex: 0x%X  |  data: 0x%X",
        node_id,
        data_to_transmit->original_data.index_,
        data_to_transmit->original_data.subindex_,
        data_to_transmit->original_data.data_);
      */
    }
  }
  return hardware_interface::return_type::OK;
}

uint32_t CustomMappingCanopenSystem::scale(const double data, const double scale_factor)
{
  return static_cast<uint32_t>(data * scale_factor);
}

double CustomMappingCanopenSystem::convert_rpm_to_rads(const uint32_t rpm)
{
  double rpm_raw = static_cast<double>(rpm);
  double rads = rpm_raw * (2 * M_PI); // 1 RPM = 2 PI rad/s
  return rads;
}

double CustomMappingCanopenSystem::convert_rads_to_rpm(const double rads)
{
  double rpm = rads/(2 * M_PI);  // 2 PI rad/s = 1 RPM
  return rpm;
}

double CustomMappingCanopenSystem::convert_rpm_to_percentage(double rpm)
{
  // RPM to 1.1 m/s: 1.1 / ((2 * 0.135) * PI) = 1.29681805482
  double percentage = rpm / 1.29681805482;
  return percentage;
}

double CustomMappingCanopenSystem::convert_to_position(double rpdo_data) 
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double CustomMappingCanopenSystem::convert_to_veloctiy(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double CustomMappingCanopenSystem::convert_to_RPM(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double CustomMappingCanopenSystem::convert_to_temperature(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double CustomMappingCanopenSystem::convert_to_switch_voltage(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

}  // namespace custom_mapping_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  custom_mapping_canopen_system::CustomMappingCanopenSystem, hardware_interface::SystemInterface)
