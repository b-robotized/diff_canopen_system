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
#include "diff_canopen_system/diff_canopen_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("DiffCanopenSystem");

auto const COMMAND_TARGET_SPEED_TAG_INDEX = "command_interface__target_speed__index";
auto const COMMAND_TARGET_SPEED_TAG_SUBINDEX = "command_interface__target_speed__subindex";
auto const STATE_VELOCITY_TAG_INDEX = "state_interface__velocity__index";
auto const STATE_VELOCITY_TAG_SUBINDEX = "state_interface__velocity__subindex";
auto const STATE_ROTOR_POSITION_TAG_INDEX = "state_interface__rotor_position__index";
auto const STATE_ROTOR_POSITION_TAG_SUBINDEX = "state_interface__rotor_position__subindex";
auto const STATE_RPM_TAG_INDEX = "state_interface__rpm__index";
auto const STATE_RPM_TAG_SUBINDEX = "state_interface__rpm__subindex";
auto const STATE_TEMPERATURE_TAG_INDEX = "state_interface__temperature__index";
auto const STATE_TEMPERATURE_TAG_SUBINDEX = "state_interface__temperature__subindex";
auto const STATE_VOLTAGE_TAG_INDEX = "state_interface__voltage__index";
auto const STATE_VOLTAGE_TAG_SUBINDEX = "state_interface__voltage__subindex";
}

namespace diff_canopen_system
{
DiffCanopenSystem::DiffCanopenSystem() : CanopenSystem() {};

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DiffCanopenSystem::on_init(
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
      if (!check_parameter_exist(info_.joints[i].parameters, COMMAND_TARGET_SPEED_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, COMMAND_TARGET_SPEED_TAG_SUBINDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_VELOCITY_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_VELOCITY_TAG_SUBINDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_ROTOR_POSITION_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_ROTOR_POSITION_TAG_SUBINDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_RPM_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_RPM_TAG_SUBINDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_TEMPERATURE_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_TEMPERATURE_TAG_SUBINDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_VOLTAGE_TAG_INDEX, info_.joints[i].name) ||
          !check_parameter_exist(info_.joints[i].parameters, STATE_VOLTAGE_TAG_SUBINDEX, info_.joints[i].name))
      {
        init_rval = CallbackReturn::ERROR;
      }
    }
  }

  // Clear all saved PDO indices
  state_pdo_indices_.clear();

  return init_rval;
}

std::vector<hardware_interface::StateInterface> DiffCanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint16_t node_id = static_cast<uint16_t>(std::stoi(info_.joints[i].parameters["node_id"], nullptr, 0));
    RCLCPP_INFO(kLogger, "State Mapping for NodeID: 0x%X are:", node_id);
    // Mapping
    uint16_t position_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[STATE_ROTOR_POSITION_TAG_INDEX], nullptr, 0));
    uint8_t position_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[STATE_ROTOR_POSITION_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "Rotor Position:  0x%X:0x%X", position_index, position_subindex);

    uint16_t velocity_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[STATE_VELOCITY_TAG_INDEX], nullptr, 0));
    uint8_t velocity_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[STATE_VELOCITY_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "Velocity:  0x%X:0x%X", velocity_index, velocity_subindex);

    uint16_t rpm_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[STATE_RPM_TAG_INDEX], nullptr, 0));
    uint8_t rpm_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[STATE_RPM_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "RPM:  0x%X:0x%X", rpm_index, rpm_subindex);

    uint16_t temperature_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[STATE_TEMPERATURE_TAG_INDEX], nullptr, 0));
    uint8_t temperature_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[STATE_TEMPERATURE_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "Temperature:  0x%X:0x%X", temperature_index, temperature_subindex);

    uint16_t voltage_index = static_cast<uint16_t>(
      std::stoi(info_.joints[i].parameters[STATE_VOLTAGE_TAG_INDEX], nullptr, 0));
    uint8_t voltage_subindex = static_cast<uint8_t>(
      std::stoi(info_.joints[i].parameters[STATE_VOLTAGE_TAG_SUBINDEX], nullptr, 0));
    RCLCPP_INFO(kLogger, "Voltage:  0x%X:0x%X", voltage_index, voltage_subindex);

    PDO_INDICES position_pdo_indices(position_index, position_subindex);
    PDO_INDICES velocity_pdo_indices(velocity_index, velocity_subindex);
    PDO_INDICES rpm_pdo_indices(rpm_index, rpm_subindex);
    PDO_INDICES tempeature_pdo_indices(temperature_index, temperature_subindex);
    PDO_INDICES voltage_pdo_indices(voltage_index, voltage_subindex);

    // Save the PDO indices
    state_pdo_indices_.emplace_back(position_pdo_indices);
    state_pdo_indices_.emplace_back(velocity_pdo_indices);
    state_pdo_indices_.emplace_back(rpm_pdo_indices);
    state_pdo_indices_.emplace_back(tempeature_pdo_indices);
    state_pdo_indices_.emplace_back(voltage_pdo_indices);

    // Make pair
    NODE_PDO_INDICES position_node_pdos  (static_cast<int>(node_id), position_pdo_indices);
    NODE_PDO_INDICES velocity_node_pdos  (static_cast<int>(node_id), velocity_pdo_indices);
    NODE_PDO_INDICES rpm_node_pdos       (static_cast<int>(node_id), rpm_pdo_indices);
    NODE_PDO_INDICES tempeature_node_pdos(static_cast<int>(node_id), tempeature_pdo_indices);
    NODE_PDO_INDICES voltage_node_pdos   (static_cast<int>(node_id), voltage_pdo_indices);

    // Intialize the value
    state_ro_.emplace(position_node_pdos  , 0.0);
    state_ro_.emplace(velocity_node_pdos  , 0.0);
    state_ro_.emplace(rpm_node_pdos       , 0.0);
    state_ro_.emplace(tempeature_node_pdos, 0.0);
    state_ro_.emplace(voltage_node_pdos   , 0.0);

    // State converter
    state_converter_.emplace(position_node_pdos  , &DiffCanopenSystem::convert_to_position      );
    state_converter_.emplace(velocity_node_pdos  , &DiffCanopenSystem::convert_to_veloctiy      );
    state_converter_.emplace(rpm_node_pdos       , &DiffCanopenSystem::convert_to_RPM           );
    state_converter_.emplace(tempeature_node_pdos, &DiffCanopenSystem::convert_to_temperature   );
    state_converter_.emplace(voltage_node_pdos   , &DiffCanopenSystem::convert_to_switch_voltage);

    // state
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &state_ro_[position_node_pdos]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &state_ro_[velocity_node_pdos]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "rpm",
      &state_ro_[rpm_node_pdos]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature",
      &state_ro_[tempeature_node_pdos]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "voltage",
      &state_ro_[voltage_node_pdos])); 
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "nmt/state",
       &canopen_data_[node_id].nmt_state.state));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffCanopenSystem::export_command_interfaces()
{
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


hardware_interface::return_type DiffCanopenSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
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

hardware_interface::return_type DiffCanopenSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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

uint32_t DiffCanopenSystem::convert_percentage_to_speed_value(const double percentage)
{
  double speed_value_raw = round(percentage * 32767);
  uint32_t speed_value = static_cast<uint32_t>(speed_value_raw);
  return speed_value;
}

double DiffCanopenSystem::convert_rpm_to_rads(const uint32_t rpm)
{
  double rpm_raw = static_cast<double>(rpm);
  double rads = rpm_raw * (2 * M_PI); // 1 RPM = 2 PI rad/s
  return rads;
}

double DiffCanopenSystem::convert_rads_to_rpm(const double rads)
{
  double rpm = rads/(2 * M_PI);  // 2 PI rad/s = 1 RPM
  return rpm;
}

double DiffCanopenSystem::convert_rpm_to_percentage(double rpm)
{
  // RPM to 1.1 m/s: 1.1 / ((2 * 0.135) * PI) = 1.29681805482
  double percentage = rpm / 1.29681805482;
  return percentage;
}

double DiffCanopenSystem::convert_to_position(double rpdo_data) 
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double DiffCanopenSystem::convert_to_veloctiy(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double DiffCanopenSystem::convert_to_RPM(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double DiffCanopenSystem::convert_to_temperature(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

double DiffCanopenSystem::convert_to_switch_voltage(double rpdo_data)
{
  // TODO(): Do the conversion here!
  return rpdo_data;
}

}  // namespace diff_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_canopen_system::DiffCanopenSystem, hardware_interface::SystemInterface)
