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
#include "diff_canopen_system/DiffCanopenSystem.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("DiffCanopenSystem");
}

namespace diff_canopen_system
{
DiffCanopenSystem::DiffCanopenSystem(): CanopenSystem() {};

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

// void DiffCanopenSystem::initDeviceContainer()
// {
//   std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"")
//   ? ""
//   : info_.hardware_parameters["master_bin"];
//
//   device_container_->init(
//     info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
//     info_.hardware_parameters["bus_config"], tmp_master_bin);
//   auto drivers = device_container_->get_registered_drivers();
//   RCLCPP_INFO(kLogger, "Number of registered drivers: '%zu'", device_container_->count_drivers());
//   for (auto it = drivers.begin(); it != drivers.end(); it++)
//   {
//     auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(it->second);
//
//     auto nmt_state_cb = [&](canopen::NmtState nmt_state, uint8_t id)
//     { canopen_data_[id].nmt_state.set_state(nmt_state); };
//     // register callback
//     proxy_driver->register_nmt_state_cb(nmt_state_cb);
//
//     auto rpdo_cb = [&](ros2_canopen::COData data, uint8_t id)
//     { canopen_data_[id].rpdo_data.set_data(data); };
//     // register callback
//     proxy_driver->register_rpdo_cb(rpdo_cb);
//
//     RCLCPP_INFO(
//       kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '0x%X'",  //
//       it->second->get_node_base_interface()->get_name(), it->first);
//   }
//
//   RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
// }

std::vector<hardware_interface::StateInterface> DiffCanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Comment this cout since hardware_interface::StateInterface does not support default constructor
  // state_interfaces.resize(info_.joints.size() * 7);

  // for (size_t i = 0; i < info_.joints.size(); ++i)
  // {
  //   if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
  //   {
  //     // skip adding canopen interfaces for non-can joints
  //     continue;
  //   }
  //
  //   const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY + std::string("_reference"), &wheel_states_[node_id].velocity_reference));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheel_states_[node_id].velocity_feedback));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, "motor_temperature", &wheel_states_[node_id].motor_temperature));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, "motor_power", &wheel_states_[node_id].motor_power));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, "battery_state", &wheel_states_[node_id].motor_battery_state));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, "error_status", &wheel_states_[node_id].error_status));
  //
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, "nmt/state", &canopen_data_[node_id].nmt_state.state));
  // }

  // Send request

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffCanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // Comment this cout since hardware_interface::CommandInterface does not support default constructor
  // command_interfaces.resize(info_.joints.size() * 5);

  // for (size_t i = 0; i < info_.joints.size(); ++i)
  // {
  //   if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
  //   {
  //     // skip adding canopen interfaces for non-can joints
  //     continue;
  //   }
  //
  //   const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));
  //
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
  //     &wheel_states_[node_id].velocity_command));
  //
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, "nmt/reset", &canopen_data_[node_id].nmt_state.reset_ons));
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, "nmt/reset_fbk", &canopen_data_[node_id].nmt_state.reset_fbk));
  //
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, "nmt/start", &canopen_data_[node_id].nmt_state.start_ons));
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, "nmt/start_fbk", &canopen_data_[node_id].nmt_state.start_fbk));
  // }
  return command_interfaces;
}


hardware_interface::return_type DiffCanopenSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Bateriespannung: 3398
  // Sollgeschwindigkeit: 3366

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffCanopenSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto drivers = device_container_->get_registered_drivers();


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

uint32_t DiffCanopenSystem::convert_percentage_to_speed_value(const double percentage) {
  double speed_value_raw = round(percentage * 32767);
  uint32_t speed_value = static_cast<uint32_t>(speed_value_raw);
  return speed_value;
}

double DiffCanopenSystem::convert_rpm_to_rads(const uint32_t rpm) {
  double rpm_raw = static_cast<double>(rpm);
  double rads = rpm_raw/30*M_PI; // = RPM/60*PI*2
  return rads;
}

// void DiffCanopenSystem::send_motor_battery_request() {
//   auto drivers = device_container_->get_registered_drivers();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
//   {
//     auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);
//
//     // Generate data request
//     if (it->second.tpdo_data.write_command())
//     {
//       // TODO(): Finish the data for request
//       it->second.tpdo_data.original_data.index_ = 0x2A6;
//       it->second.tpdo_data.original_data.subindex_ = 0;
//       it->second.tpdo_data.original_data.data_ = 0;
//
//       // it->second.tpdo_data.prepare_data();
//       proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
//       // Debug Message
//       RCLCPP_INFO(kLogger, "This is a debug message in send_motor_battery_request().....");
//       RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//       it->first,
//       it->second.tpdo_data.original_data.index_,
//       it->second.tpdo_data.original_data.subindex_,
//       it->second.tpdo_data.original_data.data_);
//       RCLCPP_INFO(kLogger, "--- END of the debug message in send_motor_battery_request()");
//     }
//   }
// }
//
// void DiffCanopenSystem::send_error_status_request() {
//   auto drivers = device_container_->get_registered_drivers();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
//   {
//     auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);
//
//     // Generate data request
//     if (it->second.tpdo_data.write_command())
//     {
//       // TODO(): Finish the data for request
//       it->second.tpdo_data.original_data.index_ = 0x1A6;
//       it->second.tpdo_data.original_data.subindex_ = 0;
//       // it->second.tpdo_data.original_data.type_ = 0;
//       it->second.tpdo_data.original_data.data_ = 0;
//
//       // it->second.tpdo_data.prepare_data();
//       proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
//       // Debug Message
//       RCLCPP_INFO(kLogger, "This is a debug message in send_error_status_request().....");
//       RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//       it->first,
//       it->second.tpdo_data.original_data.index_,
//       it->second.tpdo_data.original_data.subindex_,
//       it->second.tpdo_data.original_data.data_);
//       RCLCPP_INFO(kLogger, "--- END of the debug message in send_error_status_request()");
//     }
//   }
// }
//
// void DiffCanopenSystem::send_motor_status_request() {
//   // Motor status via (T)PDO eg 3A6# XX XX YY  YY ZZ ZZ ZZ 00
//   // ( XX XX= Motor RPM; YY YY = temperatur, ZZ ZZ ZZ= Motor Power
//   auto drivers = device_container_->get_registered_drivers();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
//   {
//     auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);
//
//     // Generate data request
//     if (it->second.tpdo_data.write_command())
//     {
//       // TODO(): Finish the data for request
//       it->second.tpdo_data.original_data.index_ = 0x3A6;
//       it->second.tpdo_data.original_data.subindex_ = 0;
//       // it->second.tpdo_data.original_data.type_ = 0;
//       it->second.tpdo_data.original_data.data_ = 0;
//
//       // it->second.tpdo_data.prepare_data();
//       proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
//       // Debug Message
//       RCLCPP_INFO(kLogger, "This is a debug message in send_motor_status_request().....");
//       RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//       it->first,
//       it->second.tpdo_data.original_data.index_,
//       it->second.tpdo_data.original_data.subindex_,
//       it->second.tpdo_data.original_data.data_);
//       RCLCPP_INFO(kLogger, "--- END of the debug message in send_motor_status_request()");
//     }
//   }
// }
//
// hardware_interface::return_type DiffCanopenSystem::read_motor_battery_states()
// {
//   send_motor_battery_request();
//   auto ret_val = CanopenSystem::read();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
//
//     RCLCPP_INFO(kLogger, "This is a debug message in read_motor_battery_states().....");
//     RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//     it->first,
//     static_cast<uint16_t>(it->second.rpdo_data.index),
//     static_cast<uint8_t>(it->second.rpdo_data.subindex),
//     static_cast<uint32_t>(it->second.rpdo_data.data));
//     RCLCPP_INFO(kLogger, "--- END of the debug message in read_motor_battery_states()");
//
//     // TODO(): Concert the RPDO data to motor battery state
//     wheel_states_[it->first].motor_battery_state = 0;
//   }
//   return ret_val;
// }
//
// hardware_interface::return_type DiffCanopenSystem::read_error_status()
// {
//   send_error_status_request();
//   auto ret_val = CanopenSystem::read();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
//
//     RCLCPP_INFO(kLogger, "This is a debug message in read_error_status().....");
//     RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//     it->first,
//     static_cast<uint16_t>(it->second.rpdo_data.index),
//     static_cast<uint8_t>(it->second.rpdo_data.subindex),
//     static_cast<uint32_t>(it->second.rpdo_data.data));
//     RCLCPP_INFO(kLogger, "--- END of the debug message in read_error_status()");
//
//     // TODO(): Concert the RPDO data to error status
//     // Error status via (T)PDO eg. 1A6# xx yy zz aa bb cc dd ee ff  or 1A7#xx yy zz aa bb ..
//     // (xx, yy, zz,... =each Byte is one error flag)
//     wheel_states_[it->first].error_status = 0;
//   }
//   return ret_val;
// }
//
// hardware_interface::return_type DiffCanopenSystem::read_motor_status()
// {
//   send_motor_status_request();
//   auto ret_val = CanopenSystem::read();
//   for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
//
//     RCLCPP_INFO(kLogger, "This is a debug message in read_motor_status().....");
//     RCLCPP_INFO(kLogger, "Iterator: %u \n Index:    %i \n Subindex: %i \n Data:     %u",
//     it->first,
//     static_cast<uint16_t>(it->second.rpdo_data.index),
//     static_cast<uint8_t>(it->second.rpdo_data.subindex),
//     static_cast<uint32_t>(it->second.rpdo_data.data));
//     RCLCPP_INFO(kLogger, "--- END of the debug message in read_motor_status()");
//
//     // TODO(): Convert the data from RPDO
//     // Motor status via (T)PDO eg 3A6# XX XX YY  YY ZZ ZZ ZZ 00
//     // ( XX XX= Motor RPM; YY YY = temperatur, ZZ ZZ ZZ= Motor Power
//     wheel_states_[it->first].velocity_state = convert_rpm_to_rads(0);
//     wheel_states_[it->first].motor_temperature = 0;
//     wheel_states_[it->first].motor_power = 0;
//   }
//   return ret_val;
// }

}  // namespace diff_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_canopen_system::DiffCanopenSystem, hardware_interface::SystemInterface)
