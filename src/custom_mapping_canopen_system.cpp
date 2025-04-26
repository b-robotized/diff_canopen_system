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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomMappingCanopenSystem::on_init(
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
    node_id_t node_id = static_cast<node_id_t>(std::stoi(joint.parameters["node_id"], nullptr, 0));  // cast from any base

    commands_[joint.name] = std::vector<InterfaceToCanOpen>();
    commands_[joint.name].reserve(joint.command_interfaces.size());
    // TODO(Dr. Denis): do the same for the `state_interfaces` but scaling is then other way around
    for (const auto & interface : joint.command_interfaces)
    {
      if (!check_parameter_exist(interface.parameters, "index", joint.name) ||
          !check_parameter_exist(interface.parameters, "subindex", joint.name))
      {
        return CallbackReturn::ERROR;
      }
      auto itf_to_canopen = InterfaceToCanOpen();
      itf_to_canopen.info = interface;
      itf_to_canopen.node_id = node_id;
      itf_to_canopen.data = std::make_shared<canopen_ros2_control::Ros2ControlCOData>();
      try {
        itf_to_canopen.data->set_data(canopen_ros2_control::COData(
          static_cast<uint16_t>(std::stoi(interface.parameters["index"], nullptr, 0)),  // cast from any base
          static_cast<uint8_t>(std::stoi(interface.parameters["subindex"], nullptr, 0)),  // cast from any base
          0.0));
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
            RCLCPP_FATAL(kLogger, "Unit '%s' unknown!", param.second.c_str(), joint.name.c_str());
            return CallbackReturn::ERROR;
          }
        }

        commands_[node_id].push_back(itf_to_canopen);
      }
    }
  }

  return init_rval;
}

std::vector<hardware_interface::StateInterface> CustomMappingCanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : states_[joint.name])
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, interface.info.name, &interface.data->data));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "nmt/state",
       &canopen_data_[node_id].nmt_state.state));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CustomMappingCanopenSystem::export_command_interfaces()
{
  // TODO(Dr. Denis): do the same for command interfaces

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"], nullptr, 0));
    RCLCPP_INFO(kLogger, "Command Mapping for NodeID: 0x%X are:", node_id);
    // Mapping - TODO(): Check interface type
    uint16_t velocity_ref_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[COMMAND_TARGET_SPEED_TAG_INDEX], nullptr, 0));
    uint8_t velocity_ref_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[COMMAND_TARGET_SPEED_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "Target Speed:  0x%X:0x%X", velocity_ref_index, velocity_ref_subindex);

    PDO_INDICES velocity_ref_indices(velocity_ref_index, velocity_ref_subindex);

    // Make pair
    NODE_PDO_INDICES velocity_ref_node_indices(node_id, velocity_ref_indices);
    velocity_command_.emplace(velocity_ref_node_indices, 0.0);

    // register the interface
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_command_[velocity_ref_node_indices]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "nmt/reset", &canopen_data_[node_id].nmt_state.reset_ons));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "nmt/reset_fbk", &canopen_data_[node_id].nmt_state.reset_fbk));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "nmt/start", &canopen_data_[node_id].nmt_state.start_ons));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "nmt/start_fbk", &canopen_data_[node_id].nmt_state.start_fbk));
  }
  return command_interfaces;
}


hardware_interface::return_type CustomMappingCanopenSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(Dr. Denis): we should be able to read directly by getting the PDO data - this is old!


  auto ret_val = CanopenSystem::read(time, period);
  // if not OK then return with error
  if (ret_val != hardware_interface::return_type::OK)
  {
    RCLCPP_ERROR(kLogger, "Error has hapend in underlaying CanopenSystem::read call. See above for more details.");
    return ret_val;
  }

  // Find a mapping between RPDOs and the state variables..
  // This for loop read the current value from the different joints.
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const uint16_t node_id = static_cast<uint16_t>(std::stoi(info_.joints[i].parameters["node_id"], nullptr, 0));

    for (auto pdo_index : state_pdo_indices_)
    {
      // Get the data from the map
      double data = canopen_data_[node_id].get_rpdo_data(pdo_index.first, pdo_index.second);

      // Convert data to desired format
      NODE_PDO_INDICES node_rpdo_indices(node_id, pdo_index); 
      double processed_state = state_converter_[node_rpdo_indices](data);

      // Write to state interface
      state_ro_[node_rpdo_indices] = processed_state;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CustomMappingCanopenSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto drivers = device_container_->get_registered_drivers();

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // TODO(dr.denis): Can we here avoid parsing of the node_id?
    const uint16_t node_id = static_cast<uint16_t>(std::stoi(info_.joints[i].parameters["node_id"], nullptr, 0));
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[node_id]);

    // reset node nmt
    if (canopen_data_[node_id].nmt_state.reset_command())
    {
      canopen_data_[node_id].nmt_state.reset_fbk = static_cast<double>(proxy_driver->reset_node_nmt_command());
      enable_write_ = true;
    }

    // start nmt
    if (canopen_data_[node_id].nmt_state.start_command())
    {
      canopen_data_[node_id].nmt_state.start_fbk = static_cast<double>(proxy_driver->start_node_nmt_command());
      enable_write_ = true;
    }
    enable_write_ = true;

    // tpdo data one shot mechanism
    if (enable_write_)
    {
      // Convert percents command to speed data
      // Command interface (rad/s) -> RPM -> Percentage -> CAN - Data

      // Maybe we do not need this mapping
      uint16_t velocity_ref_index = static_cast<uint16_t>(
        std::stoi(info_.joints[i].parameters[COMMAND_TARGET_SPEED_TAG_INDEX], nullptr, 0));
      uint8_t velocity_ref_subindex = static_cast<uint8_t>(
        std::stoi(info_.joints[i].parameters[COMMAND_TARGET_SPEED_TAG_SUBINDEX], nullptr, 0));

      // Make pair
      PDO_INDICES velocity_ref_indices(velocity_ref_index, velocity_ref_subindex);
      NODE_PDO_INDICES velocity_ref_node_indices(node_id, velocity_ref_indices);

      // Get rads from command interaface and then convert to RPM
      double rpm = convert_rads_to_rpm(velocity_command_[velocity_ref_node_indices]);

      // TODO(): We need PRM -> Percentage
      double percentage = convert_rpm_to_percentage(rpm);

      // Prepare the data
      canopen_data_[node_id].tpdo_data.index = velocity_ref_index;
      canopen_data_[node_id].tpdo_data.subindex = velocity_ref_subindex;
      canopen_data_[node_id].tpdo_data.data = convert_percentage_to_speed_value(percentage);
      canopen_data_[node_id].tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(canopen_data_[node_id].tpdo_data.original_data);
      // Debug Message
      // RCLCPP_INFO(kLogger, "This is a debug message in HW-write().....");
      // RCLCPP_INFO(kLogger, "Iterator: 0x%X; Index: 0x%X; Subindex: 0x%X; Data: %u",
      //   node_id,
      //   canopen_data_[node_id].tpdo_data.original_data.index_,
      //   canopen_data_[node_id].tpdo_data.original_data.subindex_,
      //   canopen_data_[node_id].tpdo_data.original_data.data_);
      // RCLCPP_INFO(kLogger, "--- END of the debug message in HW-write()");
    }
  }

  return hardware_interface::return_type::OK;
}

uint32_t CustomMappingCanopenSystem::convert_percentage_to_speed_value(const double percentage)
{
  double speed_value_raw = round(percentage * 32767);
  uint32_t speed_value = static_cast<uint32_t>(speed_value_raw);
  return speed_value;
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
